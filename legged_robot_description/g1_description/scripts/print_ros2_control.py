import argparse

import xml.etree.ElementTree as ET

def print_joint_names(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints = root.findall('joint')
    print("Joint names in URDF:")
    for joint in joints:
        name = joint.attrib.get('name')
        if name:
            print(name)

def print_ros2_control_config(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints = root.findall('joint')
    joint_cnt = 0
    for joint in joints:
        name = joint.attrib.get('name')
        if not name:
            print("Joint without name found")
            break
        
        type_name = joint.attrib.get('type', 'unknown')
        if type_name != "revolute":
            continue

        joint_cnt += 1

        limit = joint.find('limit')
        if limit is not None:
            lower = limit.attrib.get('lower', '')
            upper = limit.attrib.get('upper', '')
            effort = limit.attrib.get('effort', '')
            velocity = limit.attrib.get('velocity', '')
        else:
            print(f"Joint {name} does not have limits defined.")
            break

        joint_elem = ET.Element('joint', attrib={'name': '${prefix}'+name})

        # set command interface
        command_interface_position_elem = ET.SubElement(joint_elem, 'command_interface', attrib={'name': 'position'})
        ET.SubElement(command_interface_position_elem, 'param', attrib={'name': 'min'}).text = lower
        ET.SubElement(command_interface_position_elem, 'param', attrib={'name': 'max'}).text = upper

        command_interface_velocity_elem = ET.SubElement(joint_elem, 'command_interface', attrib={'name': 'velocity'})
        ET.SubElement(command_interface_velocity_elem, 'param', attrib={'name': 'min'}).text = str(-float(velocity))
        ET.SubElement(command_interface_velocity_elem, 'param', attrib={'name': 'max'}).text = str(float(velocity))

        command_interface_effort_elem = ET.SubElement(joint_elem, 'command_interface', attrib={'name': 'effort'})
        ET.SubElement(command_interface_effort_elem, 'param', attrib={'name': 'min'}).text = str(-float(effort))
        ET.SubElement(command_interface_effort_elem, 'param', attrib={'name': 'max'}).text = str(float(effort))

        # set state interface
        state_interface_position_elem = ET.SubElement(joint_elem, 'state_interface', attrib={'name': 'position'})
        ET.SubElement(state_interface_position_elem, 'param', attrib={'name': 'initial_value'}).text = str(0.0)
        
        state_interface_velocity_elem = ET.SubElement(joint_elem, 'state_interface', attrib={'name': 'velocity'})

        state_interface_effort_elem = ET.SubElement(joint_elem, 'state_interface', attrib={'name': 'effort'})

        xml_str = ET.tostring(joint_elem, encoding='unicode')
        print(xml_str)

    print(joint_cnt)



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Print joint names from a URDF file.")
  parser.add_argument("urdf_file", nargs='?', default="../urdf/g1_29dof_lock_waist_rev_1_0.urdf", help="Path to the URDF file")
  args = parser.parse_args()
  print_ros2_control_config(args.urdf_file)