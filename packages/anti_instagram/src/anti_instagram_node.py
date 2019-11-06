#!/usr/bin/env python

import rospy
import threading


from duckietown import DTROS
from anti_instagram.AntiInstagram import AntiInstagram
from cv_bridge import CvBridge
from duckietown_utils.jpg import bgr_from_jpg
from sensor_msgs.msg import CompressedImage


class AntiInstagramNode(DTROS):
    """Color correction.

    The node corrects the image colors to better separate white yellow and red.

    The configuration parameters can be changed dynamically while the node is running via `rosparam set` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
    ~ai_interval (:obj:`float`): The time interval between two corrections, default is 10s
    ~cb_percentage (:obj:`float`): The percentage of color balance TODO figure out what is it
    ~scale_percent (:obj:`float`): Output scale TODO figure out what is it
    ~resize (:obj:`float`): Calculation size, TODO figure out what is it
    """
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AntiInstagramNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary
        self.parameters['~ai_interval'] = None
        self.parameters['~cb_percentage'] = None
        self.parameters['~scale_percent'] = None
        self.parameters['~resize'] = None
        self.updateParameters()

        # Setup AntiInstagram
        self.image_lock = threading.Lock()
        self.ai = AntiInstagram()

        self.parameters['~ai_interval'] = self.setup_parameter("~ai_interval", 10)
        self.parameters['~cb_percentage'] = self.setup_parameter("~cb_percentage", 0.8)  # XXX: change in all launch files
        self.parameters['~scale_percent'] = self.setup_parameter("~scale_percent", 0.4)  # XXX: change in all launch files
        self.calculation_scale = self.setup_parameter("~resize", 0.2)

        self.bridge = CvBridge()

        self.image = None

        rospy.Timer(rospy.Duration(self.parameters['~ai_interval']), self.calculate_new_parameters)

        self.uncorrected_image_subscriber = rospy.Subscriber(
                                                '~uncorrected_image/compressed',
                                                CompressedImage,
                                                self.process_image,
                                                buff_size=921600,
                                                queue_size=1)

        self.corrected_image_publisher = rospy.Publisher(
                                             "~corrected_image/compressed",
                                             CompressedImage,
                                             queue_size=1)

        self.log("Initialized.")

    def process_image(self, image_msg):
        try:
            self.image_lock.acquire()
            image = bgr_from_jpg(image_msg.data)
            self.image = image
            self.image_lock.release()
        except ValueError as e:
            self.log('Anti_instagram cannot decode image: %s' % e)
            self.image_lock.release()
            return

        color_balanced_image = self.ai.apply_color_balance(image,
                                   self.parameters['~scale_percent'])

        if color_balanced_image is None:
            self.calculate_new_parameters(None)
            return

        corrected_image = self.bridge.cv2_to_compressed_imgmsg(
                              color_balanced_image)
        corrected_image.header.stamp = image_msg.header.stamp
        self.corrected_image_publisher.publish(corrected_image)

    def calculate_new_parameters(self, event):
        self.image_lock.acquire()
        image = self.image
        self.image_lock.release()

        if image is None:
            self.log("[%s] Waiting for first image!" % self.node_name)
            return

        self.ai.calculate_color_balance_thresholds(image,
                                                   self.parameters['~resize'],
                                                   self.parameters['~cb_percentage'])

        self.log("[%s] New parameters computed" % self.node_name)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        self.log("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    # Initialize the node
    anti_instagram_node = AntiInstagramNode(node_name='anti_instagram_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
