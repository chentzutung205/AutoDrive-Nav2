import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import time

GOALS = [((0.94, 0.17, 0.0), (0.0, 0.0, 0.0, 1.0)),
         ((0.38, 0.66, 0.0), (0.0, 0.0, 0.0, 1.0)),
         ((1.12, 1.48, 0.0), (0.0, 0.0, 0.0, 1.0))]  # Example goals

class MapToGoal(Node):
    def __init__(self, goals):
        super().__init__('map_to_goal')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = goals

        self.distance_limit = 0.2
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        self._action_client.wait_for_server()

        self.feedback_counter = 0

        self.publish_goal()


    def publish_goal(self):
        
        if not self.goals:
            self.get_logger().info('All goals have been published. Shutting down...')
            self.destroy_node()
            rclpy.shutdown()
            return

        goal_tuple = self.goals.pop(0)
        print("Goal: ", goal_tuple)
        goal_pose = self.create_pose(goal_tuple)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info(f'Goal sent: position={goal_tuple[0]}, orientation={goal_tuple[1]}')


    def create_pose(self, goal_tuple):
        goal_pose = PoseStamped()
        #goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = goal_tuple[0]
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = goal_tuple[1]

        self.get_logger().info(
        f'Publishing goal with position: (x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}, z={goal_pose.pose.position.z}) '
        f'and orientation: (x={goal_pose.pose.orientation.x}, y={goal_pose.pose.orientation.y}, '
        f'z={goal_pose.pose.orientation.z}, w={goal_pose.pose.orientation.w})'
    )
        return goal_pose


    def feedback_callback(self, feedback_msg):
        self.feedback_counter += 1

        if self.feedback_counter <= 3:
            self.get_logger().info('Ignoring initial zero distance feedback.')
            return

        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance to goal: {feedback.distance_remaining}')

        print('distance to go',feedback.distance_remaining)
        if feedback.distance_remaining < self.distance_limit and self.goals:
            self.get_logger().info('Close to the goal. Publishing the next goal.')
            self.publish_goal()
            time.sleep(8)


def main(args=None):
    rclpy.init(args=args)
    try:
        map_to_goal_node = MapToGoal(GOALS)
        rclpy.spin(map_to_goal_node)
    except KeyboardInterrupt:
        pass
    finally:
        map_to_goal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
