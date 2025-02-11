import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from bot_interfaces.msg import OrderStatus
import time

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.navigator = BasicNavigator()

        # Subscribe to order status updates
        self.order_subscriber = self.create_subscription(
            OrderStatus, '/order_status', self.process_orders, 10)

        self.orders = {}  # Dictionary to store order states
        self.get_logger().info("Robot Navigator is now active.")

        # Define key locations
        self.poses = {
            "home": self.set_pose(0.0, 0.0),
            "kitchen": self.set_pose(4.0, 2.0),
            "table1": self.set_pose(4.4, -4.0),
            "table2": self.set_pose(4.8, -8.0),
            "table3": self.set_pose(3.3, -8.0)
        }
        self.navigator.setInitialPose(self.poses['home'])

    def set_pose(self, x, y):
        """Helper function to set robot poses"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        return pose

    def process_orders(self,msg):
        """Process order status updates"""
        self.deliver_order(int(msg.table_number))
        if msg.status == "canceled":
            self.handle_cancellation(int(msg.table_number))

    def deliver_order(self, table_number):
        """Handles full delivery flow with timeout and home return"""
        self.get_logger().info(f"Processing order for Table {table_number}...")

        # Step 1: Go to Kitchen
        if not self.navigate_to("kitchen"):
            return self.return_home()

        # Step 2: Wait for kitchen confirmation (10 sec timeout)
        if not self.check_confirmation(table_number, 10):
            self.get_logger().info(f"Order for Table {table_number} timed out. Returning home.")
            return self.return_home()

        # Step 3: Go to the table
        table_key = f"table{table_number}"
        if not self.navigate_to(table_key):
            return self.return_home()

        # Step 4: Check confirmation at table
        if not self.check_confirmation(table_number, 10):
            self.get_logger().info(f"Customer at Table {table_number} did not confirm. Returning home.")
            return self.return_home()

        # Step 5: Order complete → Return home
        self.get_logger().info(f"Order for Table {table_number} completed. Returning home.")
        self.update_order_status(table_number, "completed")
        self.return_home()

    def handle_cancellation(self, table_number):
        """Handles cancellation by going to the Kitchen first, then Home"""
        self.get_logger().warn(f"Order {table_number} was canceled. Adjusting route.")

        if not self.navigate_to("kitchen"):  # Go to Kitchen first
            return self.return_home()

        self.get_logger().info(f"Order {table_number} is now officially canceled. Returning home.")
        self.return_home()

    def navigate_to(self, location):
        """Moves the robot to a specified location"""
        self.get_logger().info(f"Navigating to {location}...")
        self.navigator.goToPose(self.poses[location])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'{location}: {feedback.distance_remaining:.2f} meters remaining.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {location} successfully.")
            return True
        else:
            self.get_logger().error(f"Failed to reach {location}.")
            return False

    def check_confirmation(self, table_number, status ,timeout_sec=10):
        """Simulates confirmation with a timeout"""
        self.get_logger().info(f"Waiting {timeout_sec} seconds for confirmation at Table {table_number}...")
        time.sleep(timeout_sec)
        if status == "confirmed":
            return True
        else:
            return False

    def return_home(self):
        """Returns the robot to home position"""
        self.get_logger().info("Returning home.")
        self.navigate_to("home")

    def update_order_status(self, table_number, status):
        """Updates order status"""
        self.orders[str(table_number)] = status
        order_status_msg = OrderStatus()
        order_status_msg.table_number = list(self.orders.keys())
        order_status_msg.status = list(self.orders.values())
        self.get_logger().info(f"Updated order status: {self.orders}")


def main(args=None):
    rclpy.init(args=args)
    robot_navigator = RobotNavigator()
    rclpy.spin(robot_navigator)
    robot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
