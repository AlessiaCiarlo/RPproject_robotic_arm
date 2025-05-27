from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

 
def generate_launch_description():
 
    # Definizione package e fileames
    urdf_package = 'robotic_arm'
    urdf_filename = 'robotic_arm.urdf'
    rviz_config_filename = 'configuration.rviz'
 
    # Path dei file
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution([pkg_share_description, 'urdf', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution([pkg_share_description, 'rviz', rviz_config_filename])
 
    # Launch configuration 
    jsp_gui = LaunchConfiguration('jsp_gui')  # per la gui con sliders che poi ho "silenziato" 
    rviz_config_file = LaunchConfiguration('rviz_config_file') # file di configurazione per rviz
    urdf_model = LaunchConfiguration('urdf_model') # robot model
    use_rviz = LaunchConfiguration('use_rviz') 
    use_sim_time = LaunchConfiguration('use_sim_time') 
 
    # Dichiarazione dei parametri launch
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui', 
        default_value='true', 
        choices=['true', 'false'],  # per abilitare o disabilitare
        description='Enable joint_state_publisher_gui')
     
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to launch RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use /clock from simulation')
     
    
    #------------------------------- NODI ----------------------------------#
 
    # joint_state_publisher (senza GUI se jsp_gui == false) per joint non-fixed 
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui))
 
    # joint_state_publisher_gui se jsp_gui == true
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        remappings= [('/joint_states','/joint_states_gui')],
        condition=IfCondition(jsp_gui))
 
    # Converte URDF in TF, sottoscritto al joint states del robot, pubblica la posa 3D 
    robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_description_content}])
       
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{
        'use_sim_time': use_sim_time}])
        
    # Nodo per la inverse kinematics 
    ik_node_cmd = Node(
        package='robotic_arm',                       
        executable='ik_node',            
        name='ik_node',
        #remappings=[('/joint_states', '/joint_states_cmd')], 
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,            
            'dh_params_file': PathJoinSubstitution([
                 pkg_share_description, 'config', 'dh_parameters.yaml'
            ])
        }]
    )
    
    #------------------ Composizione della LaunchDescription ----------------------#
   
    ld = LaunchDescription()
 
    # Aggiunta dei parametri launch definiti sopra
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
 
    # Aggiunta dei nodi
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(ik_node_cmd)
 
    return ld


