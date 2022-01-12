# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('py_pub')
        self.declare_parameter('topic_name', 'helloworld')
        self.declare_parameter('topic_period',0.5)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        topic_period = self.get_parameter('topic_period').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(topic_period, self.timer_callback)
        self.i = 0
        self.get_logger().info("Initialized [py_pub] with topic name: '%s' (Period: %.3fs)"%(topic_name,topic_period))

    def timer_callback(self):
        msg = String()
        msg.data = '[ PY] Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Sent : "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
