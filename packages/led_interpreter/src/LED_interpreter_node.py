#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, Vector2D, AprilTagDetection, AprilTagDetectionArray, LEDDetection, \
                                LEDDetectionArray, LEDDetectionDebugInfo, SignalsDetection


class LEDInterpreterNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LEDInterpreterNode, self).__init__(node_name=node_name)

        self.trafficLightIntersection = False

        self.log('[%s] Intersection Type: %s'%(self.node_name, rospy.get_param("~intersectionType")))

        if rospy.get_param("~intersectionType") == "trafficLight":
            self.trafficLightIntersection = True

        self.parameters['geom__mask'] = None
        self.parameters['freq_threshold'] = None
        self.updateParameters()

        self.protocol = rospy.get_param("~LED_protocol")

        self.signal_frequencies = self.protocol['frequencies'].values()

        # Initialize the standard output message
        self.front = SignalsDetection.NO_CAR
        self.right = SignalsDetection.NO_CAR
        self.left = SignalsDetection.NO_CAR
        self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT

        # Publishers
        self.pub_interpret = rospy.Publisher("~signals_detection", SignalsDetection, queue_size=1)

        # Subscribers
        self.sub_leds = rospy.Subscriber("~raw_led_detection", LEDDetectionArray, self.interpreter, queue_size=1)

        self.log("Initialized.")

    def interpreter(self, msg):
        self.log("[%s] Read a message from Detector" % self.node_name)
        # Initialize the standard output message
        self.front = SignalsDetection.NO_CAR
        self.right = SignalsDetection.NO_CAR
        self.left = SignalsDetection.NO_CAR
        self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT

        # TL intersection
        if self.trafficLightIntersection:
            for det in msg.detections:
                self.log("[%s]:\n pixel = %f\n top bound = %f\n measured frequence=%f\n go frequency =%f" %
                         (self.node_name, det.pixels_normalized.y, self.parameters['geom_mask']['top'],
                          det.frequency, self.lightGo))
                # Traffic light signal detection
                if det.pixels_normalized.y < self.parameters['geom_mask']['top']:
                    # Check if valid signal
                    if abs(det.frequency - self.protocol['signals']['TL_GO']) < self.freq_threshold:
                        self.traffic_light_state = SignalsDetection.GO
                        break
                    else:
                        self.traffic_light_state = SignalsDetection.STOP
                        break

        # Stop sign intersection
        else:
            for det in msg.detections:
                self.log("[%s]:\n pixel = %f\n right bound = %f\n measured frequence=%f\n" %
                         (self.node_name, det.pixels_normalized.x,
                          self.parameters['geom_mask']['right'], det.frequency))
                # Front signal detection
                if (det.pixels_normalized.x < self.parameters['geom_mask']['right'] and
                    det.pixels_normalized.y > self.parameters['geom_mask']['top']):
                    # Extract signal
                    detected_freq = det.frequency
                    for freq in self.signal_frequencies:
                        if abs(freq - detected_freq) < self.freq_threshold:
                            signals = [k for (k, v) in self.protocol['signals'].items() if v == freq]
                            self.front = signals[0]
                            break

                # Right signal detection
                if (det.pixels_normalized.x > self.parameters['geom_mask']['right'] and
                    det.pixels_normalized.y > self.parameters['geom_mask']['top']):
                    # Extract signal
                    detected_freq = det.frequency
                    for freq in self.signal_frequencies:
                        if abs(freq - detected_freq) < self.freq_threshold:
                            signals = [k for (k, v) in self.protocol['signals'].items() if v == freq]
                            self.right = signals[0]
                            break

        self.log("[%s] The observed LEDs are:\n Front = %s\n Right = %s\n Traffic light state = %s" %
                 (self.node_name, self.front, self.right,self.traffic_light_state))

        self.pub_interpret.publish(SignalsDetection(front=self.front, right=self.right, left=self.left,
                                                    traffic_light_state=self.traffic_light_state))

    def check_valid_area(self, pixel_pair, area):
        if area == 'traffic_light':
            return pixel_pair.y < self.cut_line['top']

        elif area == 'area_front':
            return pixel_pair.x < self.cut_line['right'] and pixel_pair.y > self.cut_line['top']

        elif area == 'area_right':
            return pixel_pair.x > self.cut_line['right'] and pixel_pair.y > self.cut_line['top']

        else:
            return False

    def onShutdown(self):
        # Reset before shutdown
        self.front = SignalsDetection.NO_CAR
        self.right = SignalsDetection.NO_CAR
        self.left = SignalsDetection.NO_CAR
        self.traffic_light_state = SignalsDetection.NO_TRAFFIC_LIGHT
        self.log("[LED interpreter] Shutdown.")


if __name__ == '__main__':
    # Initialize the node
    led_interpreter_node = LEDInterpreterNode(node_name='led_interpreter_node')
    # Setup proper shutdown behavior
    rospy.on_shutdown(led_interpreter_node.onShutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
