
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

    # Sends a log message to the logging system
    def log(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)

    # Sends a warning message to the logging system
    def warn(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)

    # Sends an error message to the logging system
    def error(self, message: str) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR
        status.name = self.name
        status.message = message
        status.hardware_id = self.id

        self.logger.publish(status)
