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
            OrderStatus, '/order_status', self.update_order_status, 10)
        # Publisher
        self.order_publisher = self.create_publisher(OrderStatus, '/order_status', 10)

        self.g_table_number = None
        self.g_status = None
        self.orders = {}

        self.get_logger().info("Robot Navigator is now active.")
        self.navigator.waitUntilNav2Active()

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
    
    def process_orders(self):
        """Process order status updates"""
        # Get the latest order status
        self.get_logger().warn(f"process_orders")
        #table_number, status = self.update_order_status()  # Calling update function here
        # if self.g_table_number is None or self.g_status is None:
        #     return  # No new order to process
        # elif self.g_status == "pending":
        self.deliver_order()
        if self.g_status == "canceled":
            self.handle_cancellation(self.g_table_number)
        self.get_logger().info(f"process_orders {self.g_table_number} → {self.g_status}")

    def deliver_order(self):
        """Handles full delivery flow with timeout and home return"""
        #self.get_logger().info(f"Processing order for Table {table_number}...")
        self.get_logger().warn(f"deliver_order {self.g_table_number} → {self.g_status}")

        # Step 1: Go to Kitchen
        self.navigate_to("kitchen")

        # Step 2: Wait for kitchen confirmation (10 sec timeout)
        if not self.check_confirmation():
            self.get_logger().info(f"check_confirmation Table {self.g_table_number} → {self.g_status}")
            self.get_logger().info(f"Order for Table {self.g_table_number} timed out. Returning home.")
            return self.return_home()

        # Step 3: Go to the table
        table_key = f"table{self.g_table_number}"
        if not self.navigate_to(table_key):
            return self.return_home()

        # Step 4: Check confirmation at table
        if not self.check_confirmation():
            self.get_logger().info(f"Customer at Table {self.g_table_number} did not confirm. Returning kitchen.")
            return self.navigate_to("kitchen")

        # Step 5: Order complete → Return home
        self.get_logger().info(f"Order for Table {self.g_table_number} completed. Returning home.")
        self.publish_status(self.g_table_number, "completed")
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

    def check_confirmation(self, timeout_sec=10):
        """Simulates confirmation with a timeout"""
        self.get_logger().info(f"Waiting {timeout_sec} seconds for confirmation at Table {self.g_table_number}...")
        time.sleep(timeout_sec)
        if self.g_status == "confirmed":
            self.get_logger().warn(f"{self.g_status} true")
            self.g_status = "pending"
            self.publish_status(self.g_table_number, "pending")
            return True
        else:
            self.get_logger().warn(f"{self.g_status} false")
            return False

    def return_home(self):
        """Returns the robot to home position"""
        self.get_logger().info("Returning home.")
        self.navigate_to("home")

    def publish_status(self, table_number, status):
        """Publishes order status updates."""
        status_msg = OrderStatus()
        status_msg.table_number = table_number
        status_msg.status = status
        self.order_publisher.publish(status_msg)
        self.get_logger().info(f"Published status: {table_number} -> {status}")

    def update_order_status(self,msg):
        """Updates order status"""
        table_number = msg.table_number  # Extract table number correctly
        status = msg.status  # Extract status correctly
        self.orders[str(table_number)] = status
        
        # Ensure proper message format
        order_status_msg = OrderStatus()
        order_status_msg.table_number = table_number  # Keep as int
        order_status_msg.status = status # Keep as string
        self.g_table_number = table_number
        self.g_status = status
        # Print only the required details
        self.get_logger().warn(f"Updated order status: Table {table_number} → {status}")
        self.process_orders()
        #return self.g_table_number, self.g_status  # Always return the latest order status




def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()
    rclpy.spin(node)  # Start event loop
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
