
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from diagnostic_msgs.msg import DiagnosticStatus

class RosLogger():
    def __init__(self, node: Node, name: str) -> None:
        self.logger = node.create_publisher(DiagnosticStatus, '/logging', 10)
        self.name = name
        self.id = "No ID"

        with open('/etc/machine-id') as id_file:
            self.id = id_file.read()[:-1]

    def log(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)

    def warn(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)

    def error(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)

def main():
    # rclpy.init()

    # device = 0
    # if len(sys.argv) > 1:
    #     device = int(sys.argv[1])
    
    # rclpy.spin(aruco_tacker)
    # rclpy.shutdown()
    pass

if __name__ == "__main__":
    main()