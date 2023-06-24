#!/usr/bin/env python
import rclpy
from exercise_interfaces.msg import Detections2d
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool


class PersonDetectionStatus(Node):
    def __init__(self):
        super().__init__('person_detection_status')
        queue_size = 10
        self.publisher = self.create_publisher(
            Bool, '/person_detector/status', queue_size)
        self.subscriber = self.create_subscription(
            Detections2d, '/person_detector/detections', self.human_detection_status, queue_size)
    
    def human_detection_status(self, detectionMsg):
        person_detected = Bool()
        if len(detectionMsg.detections) != 0:
            person_detected.data = True
            print("person detected")
        else:
            person_detected.data = False
        self.publisher.publish(person_detected)


def main(args=None):
    rclpy.init(args=args)
    person_detection_status = PersonDetectionStatus()
    try:
        rclpy.spin(person_detection_status)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        person_detection_status.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
