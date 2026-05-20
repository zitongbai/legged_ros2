import pathlib
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_msgs.msg import TFMessage


PARAMS_FILE = pathlib.Path(__file__).resolve().parent / "ground_odom_tf.yaml"


def static_tf_node(name, x, y, z, child_frame):
    return launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base",
            "--child-frame-id",
            child_frame,
        ],
        output="screen",
    )


def generate_test_description():
    foot_publishers = [
        static_tf_node("fl_foot_tf", 0.2, 0.1, -0.3, "FL_foot"),
        static_tf_node("fr_foot_tf", 0.2, -0.1, -0.3, "FR_foot"),
        static_tf_node("rl_foot_tf", -0.2, 0.1, -0.3, "RL_foot"),
        static_tf_node("rr_foot_tf", -0.2, -0.1, -0.3, "RR_foot"),
    ]

    ground_odom_node = launch_ros.actions.Node(
        package="legged_mapping",
        executable="ground_odom_tf_node",
        name="ground_odom_tf_node",
        parameters=[str(PARAMS_FILE)],
        output="screen",
    )

    return launch.LaunchDescription(
        foot_publishers
        + [
            ground_odom_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestGroundOdomTfNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def _receive_transform_map(self):
        node = rclpy.create_node("ground_odom_tf_test")
        received_messages = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        subscription = node.create_subscription(
            TFMessage,
            "/tf_static",
            received_messages.append,
            qos,
        )

        deadline = time.time() + 8.0
        transform_map = {}
        expected_pairs = {
            ("odom", "initial_base"),
            ("initial_base", "camera_init"),
            ("body", "base"),
        }
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            for message in received_messages:
                for transform in message.transforms:
                    transform_map[
                        (transform.header.frame_id, transform.child_frame_id)
                    ] = transform
            if expected_pairs.issubset(transform_map.keys()):
                break

        node.destroy_subscription(subscription)
        node.destroy_node()
        self.assertTrue(
            expected_pairs.issubset(transform_map.keys()),
            "Did not receive expected ground odom static transforms",
        )
        return transform_map

    def test_publishes_ground_odom_transform(self):
        transforms = self._receive_transform_map()
        ground_tf = transforms[("odom", "initial_base")]

        translation = np.array(
            [
                ground_tf.transform.translation.x,
                ground_tf.transform.translation.y,
                ground_tf.transform.translation.z,
            ]
        )
        rotation = np.array(
            [
                ground_tf.transform.rotation.x,
                ground_tf.transform.rotation.y,
                ground_tf.transform.rotation.z,
                ground_tf.transform.rotation.w,
            ]
        )

        self.assertTrue(np.allclose(translation, [0.0, 0.0, 0.32], atol=1e-9))
        self.assertTrue(
            np.allclose(rotation, [0.0, 0.0, 0.0, 1.0], atol=1e-9)
            or np.allclose(rotation, [0.0, 0.0, 0.0, -1.0], atol=1e-9)
        )

    def test_preserves_calibrated_tracking_origin_transform(self):
        transforms = self._receive_transform_map()
        calibration_tf = transforms[("initial_base", "camera_init")]
        self.assertAlmostEqual(calibration_tf.transform.translation.x, 0.1)
        self.assertAlmostEqual(calibration_tf.transform.translation.y, 0.2)
        self.assertAlmostEqual(calibration_tf.transform.translation.z, 0.3)
