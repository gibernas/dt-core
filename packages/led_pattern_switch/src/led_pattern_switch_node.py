#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, BoolStamped
from std_msgs.msg import String

from duckietown import DTROS


class LEDPatternSwitchNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LEDPatternSwitchNode, self).__init__(node_name=node_name)

        self.log("Initializing... ")

        # Read parameters
        self.parameters = None
        self.mappings = rospy.get_param("~mappings")
        source_topic_dict = rospy.get_param("~source_topics")
        self.current_src_name = "joystick"  # by default if fsm is missing

        # Publishers
        self.pub_cmd = rospy.Publisher("~change_color_pattern", String, queue_size=1)
        
        # Subscribers
        self.sub_fsm_state = rospy.Subscriber(rospy.get_param("~mode"), FSMState, self.cbFSMState)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)

        self.sub_dict = dict()
        for src_name, topic_name in source_topic_dict.items():
            self.sub_dict[src_name] = rospy.Subscriber(topic_name, String, self.cb_msg_in, callback_args=src_name)

        self.log("Initialized. ")

    def cbFSMState(self, fsm_state_msg):
        self.current_src_name = self.mappings.get(fsm_state_msg.state)
        if self.current_src_name is None:
            self.log("FSMState %s not handled. No msg pass through the switch." % fsm_state_msg.state, 'warn')
        else: 
            self.log("Led pattern switched to %s in state %s." %
                     (self.current_src_name, fsm_state_msg.state))

    def cb_msg_in(self, msg, src_name):

        if src_name == self.current_src_name:
            # self.log("[%s] %s callback matches, publishing" % (self.node_name,src_name))
            self.pub_cmd.publish(msg)
        else:
            # self.log("[%s] %s callback does not match, not publishing" % (self.node_name,src_name))
            return

    def on_shutdown(self):
        self.log("[%s] Shutting down." % self.node_name)


if __name__ == '__main__':
    # Initialize the node
    led_pattern_switch_node = LEDPatternSwitchNode(node_name='led_pattern_switch')
    # Setup proper shutdown behavior
    rospy.on_shutdown(led_pattern_switch_node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

