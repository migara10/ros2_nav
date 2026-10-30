import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GotoPoseClient(Node):

    def __init__(self):
        super().__init__('goto_pose_client')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def send_goal(self, x, y, yaw):
        # Nav2 action server සූදානම් වන තෙක් රැඳී සිටීම
        self.get_logger().info('Waiting for ' \
                               '\'navigate_to_pose\' action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Target Coordinates (x, y) සකස් කිරීම
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Yaw (rotation) එක Quaternion බවට හැරවීම
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Sending Goal -> X: {x}, Y: {y}, Yaw: {yaw}'
        )

        # Goal එක යැවීම සහ ප්‍රතිඵලය බලා සිටීම
        self._send_goal_future = (
            self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # රොබෝවරයා ගමන් කරන අතරතුර ලැබෙන feedback
        feedback = feedback_msg.feedback
        # ඔබට අවශ්‍ය නම් මෙහිදී current distance ප්‍රදර්ශනය කළ හැක

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached successfully!')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # මෙහි ඔබ යමට කැමති X, Y සහ Yaw (Radians වලින්) අගයන් ලබා දෙන්න
    target_x = 1.0  # මීටර් වලින්
    target_y = 0.5  # මීටර් වලින්
    target_yaw = 0.0  # අංශක වලින් නම්: math.radians(90)

    goto_pose_client = GotoPoseClient()
    goto_pose_client.send_goal(target_x, target_y, target_yaw)

    rclpy.spin(goto_pose_client)


if __name__ == '__main__':
    main()