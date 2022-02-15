# Node for spawning the desired ezrassor rover model using the gazebo spawn_entity service

import os
import rclpy
import sys
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
from transforms3d.euler import euler2quat
import xacro

# Main function that runs the node
def main(args=None):

    # Initalize rclpy and the node for spawning the rover then set up spawn entity client
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    # Assign passed launch arguments for use through the node
    rover_model = sys.argv[1]
    robot_count = sys.argv[2]
    spawn_x = sys.argv[3]
    spawn_y = sys.argv[4]
    spawn_z = sys.argv[5]
    # Quaternion used during spawning that is created from the roll, yaw, and pitch passed as launch arguments
    quat = euler2quat(float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))

    # Get the desired models uri based on the rover_model variable
    pkg_ezrassor_arm_v2 = get_package_share_directory('ezrassor_arm_v2')
    if rover_model == 'arm':
        model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor_arm_v2.xacro')
    else:
        model_uri = os.path.join(pkg_ezrassor_arm_v2, 'resource/ezrassor.xacro')

    # Load the content of that file into xml for the spawn entity request to read
    ezrassor_description_config = xacro.process_file(model_uri)
    content = ezrassor_description_config.toxml()

    # Create the spawn entity request including its spawn position and orientation
    req = SpawnEntity.Request()
    req.name = 'ezrassor'
    req.xml = content
    req.robot_namespace = "/ezrassor"
    req.reference_frame = "world"
    req.initial_pose.position.x = float(spawn_x)
    req.initial_pose.position.y = float(spawn_y)
    req.initial_pose.position.z = float(spawn_z)
    req.initial_pose.orientation.x = quat[1]
    req.initial_pose.orientation.y = quat[2]
    req.initial_pose.orientation.z = quat[3]
    req.initial_pose.orientation.w = quat[0]

    # Wait for the spawn entity service to become available
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, watiting....')

    # Call the sevice and wait for the result
    res = cli.call_async(req)
    rclpy.spin_until_future_complete(node, res)

    # If the res is not null, service completed successfully
    if res.result() is not None:
        node.get_logger().info('Result: ' + str(res.result().success) + "/Message: " + res.result().status_message)
    # Else log that the service call failed
    else:
        node.get_legger().info('Service call failed: %r' % (res.exception(),))

    # Destroy/kill node and close the rclpy instance
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()