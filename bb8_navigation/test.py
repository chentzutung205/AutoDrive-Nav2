# This script is to set the goal one-by-one manually

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose  # Use the appropriate action message

# Points to navigate to
GOALS = [((2.01, -1.01, 0.00), (0.0, 0.0, 0.0, 1.0)), ((3.06, -0.04, 0.00), (0.0, 0.0, 0.0, 1.0)), ((4.61, -0.68, 0.00), (0.0, 0.0, 0.0, 1.0))] # shall be changed
desired_distance = 0.05

class MapToGoal(Node):
    
    def __init__(self):
        
        super().__init__('map_to_goal')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')  # Use the correct action name
        
        self.numofgoal = 0
        self.goals = GOALS


    def send_goal(self):
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'  # Set the correct frame ID
        goal_msg.pose.pose.position.x = 2.01  # Set your goal position
        goal_msg.pose.pose.position.y = -1.01
        goal_msg.pose.pose.position.z = 0.00
        goal_msg.pose.pose.orientation.x = 0.00  # Set your goal orientation
        goal_msg.pose.pose.orientation.y = 0.00
        goal_msg.pose.pose.orientation.z = 0.00
        goal_msg.pose.pose.orientation.w = 1.00

        if not self.goals:
            self.get_logger().info('All goals have been published.')
            self.destroy_node()
            rclpy.shutdown()
            return

        self.goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)


    def feedback_callback(self, feedback_msg):
        
        distance_to_goal = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance to goal: {distance_to_goal}")

        if (distance_to_goal < desired_distance):
            print("Switch to next goal!")
            self.counter()


    def counter(self):
        self.numofgoal += 1


def main(args=None):
    rclpy.init(args=args)
    map_to_goal = MapToGoal()

    while rclpy.ok():
        map_to_goal.send_goal()
        rclpy.spin_once(map_to_goal)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
