import os
import rclpy
import sys
from gazebo_msgs.srv import SpawnEntity

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]

    req = SpawnEntity.Request()
    req_name = 'ezrassor'
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