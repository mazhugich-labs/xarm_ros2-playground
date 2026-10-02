"""Compare the simulation model to the upstream real-driver model."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def expand(package, relative_path):
    path = Path(get_package_share_directory(package)) / relative_path
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def normalized(element):
    """Ignore XML formatting and the package/file URI difference for meshes."""
    attributes = dict(element.attrib)
    if element.tag == 'mesh':
        filename = attributes['filename']
        if filename.startswith('package://xarm_description/'):
            filename = filename.replace(
                'package://xarm_description/',
                get_package_share_directory('xarm_description') + '/',
                1,
            )
        else:
            filename = filename.removeprefix('file://')
        assert Path(filename).is_file(), filename
        attributes['filename'] = filename
    return (
        element.tag, attributes, (element.text or '').strip(),
        [normalized(child) for child in element],
    )


def test_model_matches_real_driver():
    simulated = expand('xarm_gazebo_driver_bringup', 'urdf/xarm7_sim.urdf.xacro')
    real = expand('xarm_description', 'urdf/xarm_device.urdf.xacro')
    for tag in ('link', 'joint'):
        assert [normalized(item) for item in simulated.findall(tag)] == [
            normalized(item) for item in real.findall(tag)
        ]
    assert simulated.find('joint[@name="world_joint"]/origin').attrib == {
        'xyz': '0 0 0', 'rpy': '0 0 0',
    }


def test_simulation_hardware_preserves_joint_interfaces():
    simulated = expand('xarm_gazebo_driver_bringup', 'urdf/xarm7_sim.urdf.xacro')
    real = expand('xarm_description', 'urdf/xarm_device.urdf.xacro')
    assert len(simulated.findall('ros2_control')) == 1
    assert simulated.findtext('ros2_control/hardware/plugin') == (
        'gazebo_ros2_control/GazeboSystem'
    )
    assert [normalized(joint) for joint in simulated.findall('ros2_control/joint')] == [
        normalized(joint) for joint in real.findall('ros2_control/joint')
    ]
    assert len(simulated.findall('gazebo/plugin')) == 1
