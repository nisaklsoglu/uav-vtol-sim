import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage


class DualCameraBridge(Node):

    def __init__(self):
        super().__init__('dual_camera_bridge')

        self.gz_node = GzNode()
        self.callbacks = []
        self.received = set()

        for name in ['vtol_1', 'vtol_2']:

            topic = f'/{name}/camera/image_raw'

            publisher = self.create_publisher(
                Image,
                topic,
                10
            )

            def callback(gz_image, pub=publisher, vehicle=name):

                msg = Image()

                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'{vehicle}_front_camera'

                msg.height = gz_image.height
                msg.width = gz_image.width

                msg.encoding = 'rgb8'
                msg.is_bigendian = 0

                msg.step = gz_image.step or gz_image.width * 3
                msg.data = gz_image.data

                pub.publish(msg)

                if vehicle not in self.received:
                    self.received.add(vehicle)
                    self.get_logger().info(
                        f'{vehicle}: ilk goruntu alindi '
                        f'{msg.width}x{msg.height}'
                    )

            self.callbacks.append(callback)

            success = self.gz_node.subscribe(
                GzImage,
                topic,
                callback
            )

            if not success:
                raise RuntimeError(
                    f'Gazebo aboneligi basarisiz: {topic}'
                )

            self.get_logger().info(
                f'Kamera baglandi: {topic}'
            )


def main():
    rclpy.init()

    node = DualCameraBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
