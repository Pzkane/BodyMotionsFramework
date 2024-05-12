import socket
from time import sleep
from typing import Optional
from rclpy.executors import List
from rclpy.logging import LoggingSeverity

from rclpy.node import Node
from std_msgs.msg import String


class SocketClientNode(Node):
    def __init__(self, host: str, port: int, data_size: int, topic_name: str, node_name:str, readline: bool = False, *args, **kwargs):
        super(SocketClientNode, self).__init__(node_name=node_name, namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__host = host
        self.__port = port
        self.__data_size = data_size
        self.__topic_name = topic_name
        self.readline = readline
        self.__init_publishers()
        self.__run()

    def __init_publishers(self):
        self.__pub_socket_data = self.create_publisher(String, self.__topic_name, 1)
        self.get_logger().info("Publishers inited!")

    def __return_before_newline(self, data: str) -> str:
        lines: List[str] = data.splitlines()
        return lines[0]

    def __run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.__host, self.__port))
            sleep(0.3)
            try:
                self.get_logger().info("Connected!")
                while True:
                    s.send(b"kick!\n")
                    data = s.recv(self.__data_size)
                    msg = String()
                    if self.readline:
                        reading = self.__return_before_newline(data.decode())
                        msg.data = reading
                    else:
                        msg.data = data.decode()
                    self.__pub_socket_data.publish(msg)
            except KeyboardInterrupt as e:
                s.shutdown(socket.SHUT_RDWR)
                s.close()
                print("Closing connection")
                raise e
