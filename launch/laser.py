from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_sweep_scanner',
      executable='l3xz_sweep_scanner_node',
      name='sweep_node',
      namespace='l3xz',
      output='screen',
      parameters=[
          {'serial_port' : '/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO004CX5-if00-port0'},
          {'rotation_speed': 1},
          {'sample_rate': 500},
          {'frame_id' : 'laser_link'}
      ]
    )
  ])
