import rclpy
from rclpy.node import Node
from bot_interfaces.msg import OrderStatus
from collections import deque

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.order_queue = deque()

        # Subscribers
        self.create_subscription(OrderStatus, '/new_orders', self.handle_new_order, 10)
        self.create_subscription(OrderStatus, '/order_confirmation', self.handle_confirmation, 10)

        # Publisher
        self.order_publisher = self.create_publisher(OrderStatus, '/order_status', 10)

        self.pending_confirmation = {}  # Store pending confirmations with timers
        self.get_logger().info("Order Manager is ready.")

    def handle_new_order(self, msg):
        """Handles new orders received from the topic."""
        table_number = msg.table_number
        action = msg.status

        if action == "placed":
            self.place_order(table_number)
        elif action == "cancelled":
            self.cancel_order(table_number)
        else:
            self.get_logger().warn(f"Invalid action received: {action}")

    def place_order(self, table_number):
        """Adds order and waits for confirmation."""
        if table_number in [1, 2, 3, "kitchen"]:
            self.order_queue.append(table_number)
            self.publish_status(table_number, "pending")
            self.get_logger().info(f"Order placed for Table {table_number}.")

            # Set a confirmation timeout of 10 seconds
            # if table_number not in self.pending_confirmation:
            #    # timer = self.create_timer(10.0, lambda: self.handle_timeout(table_number))
            #     self.pending_confirmation[table_number] = timer
        else:
            self.get_logger().warn(f"Invalid table number: {table_number}")

    def handle_confirmation(self, msg):
        """Handles order confirmation."""
        table_number = msg.table_number
        confirmed = msg.status == "confirmed"
        self.get_logger().warn(f"msg {msg.status}")

        
        # if table_number in self.pending_confirmation:
        #     # Stop the timer
        #     self.destroy_timer(self.pending_confirmation[table_number])
        #     del self.pending_confirmation[table_number]

        if confirmed:
            self.publish_status(table_number, "confirmed")
            self.get_logger().info(f"Order {table_number} confirmed.")
        else:
            self.publish_status(table_number, "cancelled")
            if table_number in self.order_queue:
                self.order_queue.remove(table_number)
            self.get_logger().info(f"Order {table_number} cancelled.")
        # else:
        #     self.get_logger().warn(f"Received confirmation for non-pending order {table_number}.")

    def handle_timeout(self, table_number):
        """Handles timeout if no confirmation is received."""
        self.get_logger().warn(f"Order for Table {table_number} timed out!")

        # Publish an update that the order has timed out
        msg = OrderStatus()
        msg.table_number = table_number
        msg.status = "timeout"
        self.order_publisher.publish(msg)



    def cancel_order(self, table_number):
        """Cancels an order if it's in the queue."""
        if table_number in self.order_queue:
            self.order_queue.remove(table_number)
            self.publish_status(table_number, "cancelled")
            self.get_logger().info(f"Order for Table {table_number} cancelled.")
        else:
            self.get_logger().warn(f"Order {table_number} not found.")

    def publish_status(self, table_number, status):
        """Publishes order status updates."""
        status_msg = OrderStatus()
        status_msg.table_number = table_number
        status_msg.status = status
        self.order_publisher.publish(status_msg)
        self.get_logger().info(f"Published status: {table_number} -> {status}")


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
