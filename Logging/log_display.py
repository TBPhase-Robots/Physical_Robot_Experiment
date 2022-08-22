

""" Displays information gathered by the ROS Logger.

It is currently used only by the wizard known as Jed.

  """

from csv import Dialect
import pipes
from subprocess import Popen
import os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from diagnostic_msgs.msg import DiagnosticStatus

class LoggerNode(Node):

    pipes: List[str] = []

    def __init__(self):
        super().__init__("logger")

        self.create_subscription(DiagnosticStatus, '/logging', self.log_callback, 10)

    def log_callback(self, msg: DiagnosticStatus):
        pipe_name = f'/tmp/pipe_{msg.name}'

        if not os.path.exists(pipe_name):
            os.mkfifo(pipe_name)

        if pipe_name not in self.pipes:
            Popen(['terminator', '-e', f"tail -f {pipe_name} | sed --unbuffered -e 's/\(LOG.*\)/\o033[39m\\1/' -e 's/\(ERROR.*\)/\o033[31m\\1/' -e 's/\(WARN.*\)/\o033[33m\\1/'"])
            # Popen(['terminator', '-e', "tail -f " + pipe_name + " | awk '/LOG/ {print \"\\033[39m\" $0 \"\\033[39m\"} /WARN/ {print \"\\033[33m\" $0 \"\\033[39m\"} /ERROR/ {print \"\\033[31m\" $0 \"\\033[39m\"}'"])
            self.pipes.append(pipe_name)

        log_code = 'LOG'

        if msg.level == DiagnosticStatus.WARN:
            log_code = 'WARN'
        elif msg.level == DiagnosticStatus.ERROR:
            log_code = 'ERROR'

        with open(pipe_name, 'w') as log_pipe:
            log_pipe.write(f'{log_code}: {msg.message}\n')



def main():

    """ Displays all captured information in the logger node."""
    rclpy.init()

    logger = LoggerNode()

    rclpy.spin(logger)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
