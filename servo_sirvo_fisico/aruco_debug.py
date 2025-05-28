import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray  # Asegúrate de tener instalado el paquete

class ArucoDebug(Node):
    def __init__(self):
        super().__init__('aruco_debug_node')
        self.sub = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',  # Asegúrate que el nombre sea exacto
            self.callback,
            10
        )
        self.get_logger().info("🟢 ArucoDebug suscrito a /marker_publisher/markers")

    def callback(self, msg):
        self.get_logger().info(f"🧲 Recibido mensaje con {len(msg.markers)} marcadores.")
        for marker in msg.markers:
            self.get_logger().info(f"➡️ ID: {marker.id}, Pos: ({marker.pose.pose.position.x:.2f}, {marker.pose.pose.position.y:.2f}, {marker.pose.pose.position.z:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
