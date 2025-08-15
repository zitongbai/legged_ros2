// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"
#include "unitree_sdk2_bridge/unitree_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  struct SimulationConfig
  {
    std::string robot = "go2";
    std::string robot_scene = "scene.xml";

    int domain_id = 1;
    std::string interface = "lo";

    int use_joystick = 0;
    std::string joystick_type = "xbox";
    std::string joystick_device = "/dev/input/js0";
    int joystick_bits = 16;

    int print_scene_information = 1;

    int enable_elastic_band = 0;
    int band_attached_link = 0;

  } config;

  using Seconds = std::chrono::duration<double>;

  using Float64MultiArray = std_msgs::msg::Float64MultiArray;
  std::shared_ptr<rclcpp::Node> node;
  Float64MultiArray terrain_heights_msg;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr terrain_heights_publisher;

  struct ElevationMap{
    int grid_num[2] = {15, 15}; // grid size for terrain heights, [0]:x, [1]:y
    double grid_size = 0.1; // grid size in meters
    double offset[2] = {0.0, 0.0}; // offset in meters, [0]:x, [1]:y
    int update_rate = 50; // update rate in Hz
    double vis_radius = 0.02; // radius of visualization spheres in meters
  } elevation_map;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (sim.use_elastic_band_ == 1)
                {
                  if (sim.elastic_band_.enable_)
                  {
                    vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    sim.elastic_band_.Advance(x, dx);

                    d->xfrc_applied[config.band_attached_link] = sim.elastic_band_.f_[0];
                    d->xfrc_applied[config.band_attached_link + 1] = sim.elastic_band_.f_[1];
                    d->xfrc_applied[config.band_attached_link + 2] = sim.elastic_band_.f_[2];
                  }
                }

                // call mj_step
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (1)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  if (config.robot == "h1" || config.robot == "g1")
  {
    config.band_attached_link = 6 * mj_name2id(m, mjOBJ_BODY, "torso_link");
  }
  else
  {
    config.band_attached_link = 6 * mj_name2id(m, mjOBJ_BODY, "base_link");
  }

  ChannelFactory::Instance()->Init(config.domain_id, config.interface);
  UnitreeSdk2Bridge unitree_interface(m, d);

  if (config.use_joystick == 1)
  {
    unitree_interface.SetupJoystick(config.joystick_device, config.joystick_type, config.joystick_bits);
  }

  if (config.print_scene_information == 1)
  {
    unitree_interface.PrintSceneInformation();
  }

  unitree_interface.Run();

  pthread_exit(NULL);
}

void quat_apply(const double *quat, const double *vec, double *result)
{
  // quat: [w, x, y, z]
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  // cross(q_xyz, v)
  double t[3] = {
      y * vec[2] - z * vec[1],
      z * vec[0] - x * vec[2],
      x * vec[1] - y * vec[0]
  };
  // t = 2 * cross(q_xyz, v)
  t[0] *= 2; t[1] *= 2; t[2] *= 2;
  // result = v + w * t + cross(q_xyz, t)
  result[0] = vec[0] + w * t[0] + (y * t[2] - z * t[1]);
  result[1] = vec[1] + w * t[1] + (z * t[0] - x * t[2]);
  result[2] = vec[2] + w * t[2] + (x * t[1] - y * t[0]);
}

void yaw_quat(const double *quat, double * result)
{
  // quat: [w, x, y, z]
  double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  double yaw = std::atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z));
  result[0] = std::cos(yaw / 2);
  result[1] = 0.0; // x component is zero
  result[2] = 0.0; // y component is zero
  result[3] = std::sin(yaw / 2); // z component is sin(yaw / 2)
  // normalize the quaternion
  double norm = std::sqrt(result[0] * result[0] + result[1] * result[1] +
                          result[2] * result[2] + result[3] * result[3]);
  result[0] /= norm; 
  result[1] /= norm;
  result[2] /= norm;
  result[3] /= norm;
}

void quat_apply_yaw(const double *quat, const double *vec, double *result)
{
  // Apply yaw rotation to the vector
  double yaw_quat_result[4];
  yaw_quat(quat, yaw_quat_result);
  quat_apply(yaw_quat_result, vec, result);
}

void *TerrainHeightSamplingThread(void *arg){

  while(1){
    if(d){
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

  mujoco::Simulate *sim = static_cast<mujoco::Simulate *>(arg);

  // build sampling x, y
  std::vector<double> x_samples(elevation_map.grid_num[0], 0.0);
  std::vector<double> y_samples(elevation_map.grid_num[1], 0.0);
  double x_start = elevation_map.offset[0] + elevation_map.grid_num[0] / 2.0 * elevation_map.grid_size;
  double y_start = elevation_map.offset[1] + elevation_map.grid_num[1] / 2.0 * elevation_map.grid_size;

  for(size_t i=0; i<x_samples.size(); ++i){
    x_samples[i] = x_start - i * elevation_map.grid_size;
  }
  for(size_t i=0; i<y_samples.size(); ++i){
    y_samples[i] = y_start - i * elevation_map.grid_size;
  }

  terrain_heights_msg.data.resize(elevation_map.grid_num[0] * elevation_map.grid_num[1]);
  terrain_heights_msg.layout.dim.resize(2);
  terrain_heights_msg.layout.dim[0].label = "x";
  terrain_heights_msg.layout.dim[0].size = elevation_map.grid_num[0];
  terrain_heights_msg.layout.dim[1].label = "y";
  terrain_heights_msg.layout.dim[1].size = elevation_map.grid_num[1];

  while (true)
  {
    double base_x = d->qpos[0];
    double base_y = d->qpos[1];
    double quat[4] = {d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]}; // w, x, y, z

    for (int i = 0; i < elevation_map.grid_num[0]; ++i)
    {
      for (int j = 0; j < elevation_map.grid_num[1]; ++j)
      {
        double sample_vec[3] = {
          x_samples[i], 
          y_samples[j],
          0.0 
        };
        double rotated_vec[3];
        quat_apply_yaw(quat, sample_vec, rotated_vec);
        double ray_start[3] = {base_x + rotated_vec[0], base_y + rotated_vec[1], 10.0};
        double dir_vec[3] = {0.0, 0.0, -1.0}; // down direction
        mjtByte geomgroup[mjNGROUP] = {1, 0, 0, 0, 0, 0}; // only geom group 0 is considered
        int bodyexclude = -1;
        int geomid[1] = {-1};

        double dist = mj_ray(m, d, ray_start, dir_vec, geomgroup, 1, bodyexclude, geomid);
        
        if(dist > 0) {
          // get terrain height
          double terrain_height = ray_start[2] - dist;
          // calculate the linear index
          int index = i * elevation_map.grid_num[1] + j;
          // prepare the terrain height message
          terrain_heights_msg.data[index] = terrain_height;
          // render in mujoco
          sim->elevation_map_vis_->GetPos()[index][0] = ray_start[0]; // x
          sim->elevation_map_vis_->GetPos()[index][1] = ray_start[1]; // y
          sim->elevation_map_vis_->GetPos()[index][2] = terrain_height; // z
        } else {
          std::cout << "Raycast failed at (" << ray_start[0] << ", " << ray_start[1] << ")" << std::endl;
        }
      }
    }

    // publish terrain heights
    terrain_heights_publisher->publish(terrain_heights_msg);

    usleep(1000000 / elevation_map.update_rate); // update rate in Hz
  }

  pthread_exit(NULL);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char **argv)
{

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // initialize ros2
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("unitree_mujoco_simulate");
  terrain_heights_publisher = node->create_publisher<Float64MultiArray>("terrain_heights", 10);

  std::string config_file = node->declare_parameter<std::string>("config_file", "");
  if (config_file.empty()){
    std::cerr << "Config file not specified. Please set the 'config_file' parameter." << std::endl;
    return 1;
  }

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);

  // Load simulation configuration
  YAML::Node yaml_node = YAML::LoadFile(config_file);
  config.robot = yaml_node["robot"].as<std::string>();
  config.robot_scene = yaml_node["robot_scene"].as<std::string>();
  config.domain_id = yaml_node["domain_id"].as<int>();
  config.interface = yaml_node["interface"].as<std::string>();
  config.print_scene_information = yaml_node["print_scene_information"].as<int>();
  config.enable_elastic_band = yaml_node["enable_elastic_band"].as<int>();
  config.use_joystick = yaml_node["use_joystick"].as<int>();
  config.joystick_type = yaml_node["joystick_type"].as<std::string>();
  config.joystick_device = yaml_node["joystick_device"].as<std::string>();
  config.joystick_bits = yaml_node["joystick_bits"].as<int>();

  sim->use_elastic_band_ = config.enable_elastic_band;

  auto elevation_map_cfg = yaml_node["elevation_map"];
  sim->render_elevation_map_ = elevation_map_cfg["render"].as<int>(0); // Enable elevation map rendering
  elevation_map.grid_num[0] = elevation_map_cfg["grid_num_x"].as<int>(15);
  elevation_map.grid_num[1] = elevation_map_cfg["grid_num_y"].as<int>(15);
  elevation_map.grid_size = elevation_map_cfg["grid_size"].as<double>(0.1);
  elevation_map.offset[0] = elevation_map_cfg["offset_x"].as<double>(0.0);
  elevation_map.offset[1] = elevation_map_cfg["offset_y"].as<double>(0.0);
  elevation_map.update_rate = elevation_map_cfg["update_rate"].as<int>(50);
  elevation_map.vis_radius = elevation_map_cfg["vis_radius"].as<double>(0.02);
  if (sim->render_elevation_map_)
  {
    uint8_t rgba[4] = {255, 0, 0, 255}; // red color
    sim->elevation_map_vis_ = std::make_unique<mj::ElevationMapVis>(
      elevation_map.grid_num[0]*elevation_map.grid_num[1],
      elevation_map.vis_radius,
      rgba
    );
  }
  yaml_node.~Node();
  
  std::string pkg_path = ament_index_cpp::get_package_share_directory("unitree_mujoco");
  string scene_path = pkg_path + "/unitree_robots/" + config.robot + "/" + config.robot_scene;
  const char *filename = scene_path.c_str();

  pthread_t unitree_thread;
  int rc = pthread_create(&unitree_thread, NULL, UnitreeSdk2BridgeThread, NULL);
  if (rc != 0)
  {
    std::cout << "Error:unable to create thread," << rc << std::endl;
    exit(-1);
  }

  pthread_t terrain_height_sampling_thread;
  rc = pthread_create(&terrain_height_sampling_thread, NULL, TerrainHeightSamplingThread, sim.get());
  if (rc != 0)
  {
    std::cout << "Error:unable to create terrain height sampling thread," << rc << std::endl;
    exit(-1);
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
