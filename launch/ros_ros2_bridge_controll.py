import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
  """Generate launch description with multiple components."""
  container = ComposableNodeContainer(
      name='rt2_assignment1_ctnr',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
        ComposableNode(
          package='rt2_assignment1',
          plugin='rt2_assignment1::Roboran_Pos_Service',
          name='ros2_position_server_node'),
        ComposableNode(
          package='rt2_assignment1',
          plugin='rt2_assignment1::Fin_State_Machi',
          name='ros2_state_machine_node')
      ],
      output='screen',
)

return launch.LaunchDescription([container])
