import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math

class ConeSubscriber(Node):
    def __init__(self):
        super().__init__('cone_subscriber')
        self.yellow_cones = []
        self.blue_cones = []
        self.yellow_subscriber = self.create_subscription(
            MarkerArray,
            'yellow_cones',
            self.yellow_callback,
            10
        )
        self.blue_subscriber = self.create_subscription(
            MarkerArray,
            'blue_cones',
            self.blue_callback,
            10
        )
        self.lines_publisher = self.create_publisher(
            MarkerArray,
            'lines',
            10
        )
        self.green_publisher = self.create_publisher(
            MarkerArray,
            'green_points',
            10
        )
        self.previous_line_ids = set()
        self.previous_green_ids = set()
        self.marker_id_counter = 0

    def yellow_callback(self, msg):
        #self.get_logger().info('Received yellow cones data')
        self.yellow_cones = msg.markers
        self.publish_lines()

    def blue_callback(self, msg):
        #self.get_logger().info('Received blue cones data')
        self.blue_cones = msg.markers
        self.publish_lines()

    def publish_lines(self):
        if not self.yellow_cones or not self.blue_cones:
            return

        lines, green_markers = self.create_lines_and_middle_points(self.yellow_cones, self.blue_cones, min_distance=0.2, max_distance=5.0)

        # Marker törlése az előző iterációból
        delete_lines = [Marker(action=Marker.DELETE, ns='lines', id=i) for i in self.previous_line_ids]
        delete_green = [Marker(action=Marker.DELETE, ns='green', id=i) for i in self.previous_green_ids]

        self.previous_line_ids = {line.id for line in lines}
        self.previous_green_ids = {green.id for green in green_markers}

        # Meglévő és új marker-ek publikálása
        lines_marker_array = MarkerArray()
        lines_marker_array.markers = delete_lines + lines

        green_marker_array = MarkerArray()
        green_marker_array.markers = delete_green + green_markers

        self.lines_publisher.publish(lines_marker_array)
        self.green_publisher.publish(green_marker_array)

        if green_markers:
            green_line_markers = self.create_green_polyline(green_markers)
            green_line_marker_array = MarkerArray()
            green_line_marker_array.markers = green_line_markers
            self.green_publisher.publish(green_line_marker_array)

        # Új vonalak a kék és sárga kúpok között
        yellow_connections = self.create_yellow_connections()
        blue_connections = self.create_blue_connections()

        if yellow_connections:
            yellow_lines_array = MarkerArray()
            yellow_lines_array.markers = yellow_connections
            self.lines_publisher.publish(yellow_lines_array)

        if blue_connections:
            blue_lines_array = MarkerArray()
            blue_lines_array.markers = blue_connections
            self.lines_publisher.publish(blue_lines_array)

    def create_lines_and_middle_points(self, yellow_cones, blue_cones, min_distance=0.2, max_distance=5.0):
        lines = []
        green_points = []

        for yellow_cone in yellow_cones:
            for blue_cone in blue_cones:
                p1 = yellow_cone.pose.position
                p2 = blue_cone.pose.position

                distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
                if min_distance <= distance <= max_distance and p2.x >= p1.x:
                    print(p1.x , p2.x)
                    line_marker = Marker()
                    line_marker.header.frame_id = yellow_cone.header.frame_id
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = 'lines'
                    line_marker.id = self.marker_id_counter
                    self.marker_id_counter += 1
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    line_marker.scale.x = 0.1  # Line width

                    line_marker.points = [p1, p2]

                    line_marker.color.r = 1.0
                    line_marker.color.g = 0.0
                    line_marker.color.b = 1.0
                    line_marker.color.a = 0.1  # Full opacity

                    line_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # 1 másodperces élettartam

                    lines.append(line_marker)

                    # Zöld pont létrehozása a lila vonal közepén
                    middle_point = Point()
                    middle_point.x = (p1.x + p2.x) / 2
                    middle_point.y = (p1.y + p2.y) / 2
                    middle_point.z = (p1.z + p2.z) / 2

                    green_marker = Marker()
                    green_marker.header.frame_id = yellow_cone.header.frame_id
                    green_marker.header.stamp = self.get_clock().now().to_msg()
                    green_marker.ns = 'green'
                    green_marker.id = self.marker_id_counter
                    self.marker_id_counter += 1
                    green_marker.type = Marker.SPHERE
                    green_marker.action = Marker.ADD
                    green_marker.pose.position = middle_point
                    green_marker.scale.x = 0.2  # Zöld pont átmérője
                    green_marker.scale.y = 0.2
                    green_marker.scale.z = 0.2
                    green_marker.color.r = 0.0  # Zöld szín
                    green_marker.color.g = 1.0
                    green_marker.color.b = 0.0
                    green_marker.color.a = 0.2  # Teljes átlátszatlanság

                    green_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # 1 másodperces élettartam

                    green_points.append(green_marker)

        return lines, green_points

    def create_yellow_connections(self):
        yellow_lines = []
        for i in range(len(self.yellow_cones) - 1):
            for j in range(i + 1, len(self.yellow_cones)):
                p1 = self.yellow_cones[i].pose.position
                p2 = self.yellow_cones[j].pose.position

                distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
                if 0.2 <= distance <= 5.0 and p1.x != 0.0 and p2.x != 0.0:  # Optional distance filter
                    yellow_line = Marker()
                    yellow_line.header.frame_id = self.yellow_cones[i].header.frame_id
                    yellow_line.header.stamp = self.get_clock().now().to_msg()
                    yellow_line.ns = 'yellow_lines'
                    yellow_line.id = self.marker_id_counter
                    self.marker_id_counter += 1
                    yellow_line.type = Marker.LINE_STRIP
                    yellow_line.action = Marker.ADD
                    yellow_line.scale.x = 0.1  # Line width

                    yellow_line.points = [p1, p2]

                    yellow_line.color.r = 1.0
                    yellow_line.color.g = 1.0
                    yellow_line.color.b = 0.0
                    yellow_line.color.a = 1.0  # Full opacity

                    yellow_line.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # 1 másodperces élettartam

                    yellow_lines.append(yellow_line)

        return yellow_lines

    def create_blue_connections(self):
        blue_lines = []
        for i in range(len(self.blue_cones) - 1):
            for j in range(i + 1, len(self.blue_cones)):
                p1 = self.blue_cones[i].pose.position
                p2 = self.blue_cones[j].pose.position

                distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
                if 0.2 <= distance <= 5.0 and p1.x != 0.0 and p2.x != 0.0:  # Optional distance filter
                    blue_line = Marker()
                    blue_line.header.frame_id = self.blue_cones[i].header.frame_id
                    blue_line.header.stamp = self.get_clock().now().to_msg()
                    blue_line.ns = 'blue_lines'
                    blue_line.id = self.marker_id_counter
                    self.marker_id_counter += 1
                    blue_line.type = Marker.LINE_STRIP
                    blue_line.action = Marker.ADD
                    blue_line.scale.x = 0.1  # Line width

                    blue_line.points = [p1, p2]

                    blue_line.color.r = 0.0
                    blue_line.color.g = 0.0
                    blue_line.color.b = 1.0
                    blue_line.color.a = 1.0  # Full opacity

                    blue_line.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # 1 másodperces élettartam

                    blue_lines.append(blue_line)

        return blue_lines

    def create_green_polyline(self, green_markers, max_distance=4.0):
        green_lines = []
        for i in range(len(green_markers) - 1):
            p1 = green_markers[i].pose.position
            p2 = green_markers[i + 1].pose.position

            distance = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
            if distance <= max_distance and p1.x != 0.0 and p2.x != 0.0:
                green_line = Marker()
                green_line.header.frame_id = green_markers[0].header.frame_id
                green_line.header.stamp = self.get_clock().now().to_msg()
                green_line.ns = 'green_line'
                green_line.id = self.marker_id_counter
                self.marker_id_counter += 1
                green_line.type = Marker.LINE_STRIP
                green_line.action = Marker.ADD
                green_line.scale.x = 0.05  # Line width

                green_line.points = [p1, p2]

                green_line.color.r = 0.0
                green_line.color.g = 1.0
                green_line.color.b = 0.0
                green_line.color.a = 0.5  # Full opacity

                green_line.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()  # 1 másodperces élettartam

                green_lines.append(green_line)

        return green_lines

def main(args=None):
    rclpy.init(args=args)
    node = ConeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
