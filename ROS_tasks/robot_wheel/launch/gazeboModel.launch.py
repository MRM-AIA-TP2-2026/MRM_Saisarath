import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro



def generate_launch_description():

    robotXacroName='differential_drive_robot'

    namePackage='robot_wheel'
    modelFileRelPath='model/robot.xacro'
    worldFileRelPath='model/empty_world.world'

    pathModelFile=os.path.join(get_package_share_directory(namePackage),modelFileRelPath)
    pathWorldFile=os.path.join(get_package_share_directory(namePackage),worldFileRelPath)


    robotDesc=xacro.process_file(pathModelFile).toxml()

    gazeboRosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))

    gazeboLaunch=IncludeLaunchDescription(gazeboRosPackageLaunch,launch_arguments={'world':pathWorldFile}.items())

    spawnModelNode= Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-topic','robot_description','-entity',robotXacroName, 
        '-x', '-1',  
        '-y', '-1',],output='screen')


    nodeRobotStatePublisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robotDesc,'use_sim_time':True}]
    )

    imuDataNode=Node(package="robot_wheel",
                     executable="imuDat",
                     name="imuDat",
                     output="screen")
    
    roverMoveNode=Node(package="robot_wheel",
                     executable="rovorMove",
                     name="rovorMove",
                     output="screen")



    
    LaunchDescriptionObject=LaunchDescription()

    LaunchDescriptionObject.add_action(gazeboLaunch)
    LaunchDescriptionObject.add_action(spawnModelNode)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(imuDataNode)
    LaunchDescriptionObject.add_action(roverMoveNode)

    return LaunchDescriptionObject


