import os
import rclpy
import sys
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

import xacro

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    rover_model = sys.argv[1]

    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    if rover_model == 'arm':
        model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor_arm_v2.xacro')
    else:
        model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')

    ezrassor_description_config = xacro.process_file(model_uri)
    content = ezrassor_description_config.toxml()

    req = SpawnEntity.Request()
    req.name = 'ezrassor'
    req.xml = content
    req.robot_namespace = ""
    req.reference_frame = "world"

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, watiting....')

    res = cli.call_async(req)
    rclpy.spin_until_future_complete(node, res)

    if res.result() is not None:
        node.get_logger().info('Result: ' + str(res.result().success) + "/Message: " + res.result().status_message)
    else:
        node.get_legger().info('Service call failed: %r' % (res.exception(),))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()