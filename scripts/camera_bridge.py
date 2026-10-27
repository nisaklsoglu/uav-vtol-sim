import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage

def main():
    rclpy.init()
    ros_node = Node('gazebo_direct_bridge')
    pub = ros_node.create_publisher(Image, '/camera/image_raw', 10)

    def cb(gz_msg):
        msg = Image()
        msg.header.stamp = ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = gz_msg.height
        msg.width = gz_msg.width
        msg.encoding = 'rgb8'
        msg.step = gz_msg.step
        msg.data = gz_msg.data
        pub.publish(msg)

    gz_node = GzNode()
    gz_node.subscribe(GzImage, '/camera/image_raw', cb)
    print('>>> ROS 2 KÖPRÜSÜ AKTİF: Kareler aktarılıyor... <<<')

    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    import time
    while True:
        time.sleep(1)

if __name__ == '__main__':
    main()
