import os
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
    #                                          description='Use simulation clock if true')
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyS3',
                                         description='usb bus name, e.g. ttyS3')
    use_mocap_arg= DeclareLaunchArgument("use_mocap",default_value='false',
                                        description='if has mocap use')
    use_uwb_arg = DeclareLaunchArgument('use_uwb', default_value='false',                                             
                                         description='if has uwb sensor to drive')
    use_flow_arg = DeclareLaunchArgument('use_flow', default_value='false',
                                     description='if has flow sensor to drive')    
#    pub_odom_arg = DeclareLaunchArgument('pub_odom', default_value='true',
 #                                        description='publish odom to base_footprint tf, true or false')
    uav_base_node = Node(
        package='uav_base',
        executable='uav_base', 
        output='screen',
        emulate_tty=True,
        parameters=[{
                # 'use_sim_time': LaunchConfiguration('use_sim_time'),
               'port_name': LaunchConfiguration('port_name'), 
  #              'correct_factor_vx': LaunchConfiguration('correct_factor_vx'), 
   #             'correct_factor_vth': LaunchConfiguration('correct_factor_vth'), 
    #            'auto_stop_on': LaunchConfiguration('auto_stop_on'), 
               'use_mocap': LaunchConfiguration('use_mocap'),
               'use_uwb': LaunchConfiguration('use_uwb'),
                'use_flow': LaunchConfiguration('use_flow'),
     #           'pub_odom': LaunchConfiguration('pub_odom'),  
   }]
 )

#    base_footprint_tf = Node(
##        package='tf2_ros',
##        executable='static_transform_publisher', 
#        emulate_tty=True,
#        arguments="0.0 0.0 0.05325 0.0 0.0 0.0 /base_footprint /base_link".split(' '))

#    imu_tf = Node(
#        package='tf2_ros',
#        executable='static_transform_publisher', 
#        emulate_tty=True,
#        arguments="0.0 0.0 0.0 0.0 0.0 0.0 /base_link /imu_link".split(' '))
        
    return LaunchDescription([
        # use_sim_time_arg,
         port_name_arg,
    #    correct_factor_vx_arg,
     #   correct_factor_vth_arg,
     #   auto_stop_on_arg,
         use_mocap_arg,
         use_flow_arg,
         use_uwb_arg,
         uav_base_node
        
  ])

