import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from os import system
import socket

class SetupUDPStream(Node):

    def __init__(self):
        super().__init__('setup_udp_stream')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('receive_port', 5000),
                ('width', 640),
                ('height', 480),
                ('fps', 30),
                ('host', None),
            ]
        )
        self.receive_port = self.get_parameter('receive_port').value
        self.host = self.get_parameter('host').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.video_mode = "-ex sports -awb off -mm matrix -w " + str(width) + " -h " + str(height) + " -fps " + str(fps) + " -b 2000000"


def main(args=None):
    rclpy.init(args=args)

    setup_udp_stream = SetupUDPStream()

    port = 10003
    size = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((setup_udp_stream.host, port))
    video_mode_str = str(setup_udp_stream.receive_port) + "," + str(setup_udp_stream.video_mode) + "\n"
    s.send(video_mode_str.encode())
    all_data = ""
    while not all_data.endswith('\n'):
        data = s.recv(size)
        all_data += data.decode('utf-8')
    s.close()
    system('hping3 -c 1 -2 -s ' + str(setup_udp_stream.receive_port) + ' -p ' + all_data.strip() + ' ' + setup_udp_stream.host)
    print('Received:', all_data)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
