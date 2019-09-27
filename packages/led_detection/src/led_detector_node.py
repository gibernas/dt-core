#!/usr/bin/env python
import rospy
import time
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import scipy.fftpack

from duckietown_utils.bag_logs import numpy_from_ros_compressed

from std_msgs.msg import Byte
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray, LEDDetectionDebugInfo, BoolStamped, SignalsDetection
from sensor_msgs.msg import CompressedImage, Image


class LEDDetectorNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LEDDetectorNode, self).__init__(node_name=node_name)

        # Needed to publish images
        self.bridge = CvBridge()

        # Add the node parameters to the parameters dictionary
        self.parameters['~capture_time'] = None
        self.parameters['~DTOL'] = None
        self.parameters['~useFFT'] = None
        self.parameters['~freqIdentity'] = None
        self.parameters['~crop_params'] = None
        self.parameters['~blob_detector_DB'] = None
        self.parameters['~blob_detector_tl'] = None
        self.updateParameters()

        self.active = True  # [INTERACTIVE MODE] Won't be overwritten if FSM isn't running, node always active
        self.first_timestamp = 0
        self.capture_finished = True
        self.t_init = None
        self.trigger = True
        self.node_state = 0
        self.data = []

        # Initialize detection
        self.right = SignalsDetection.NO_CAR
        self.front = SignalsDetection.NO_CAR
        self.traffic_light = SignalsDetection.NO_TRAFFIC_LIGHT
        self.left = "UNKNOWN"

        # Create Blob detector parameter objects
        bd_param_db = cv2.SimpleBlobDetector_Params()
        bd_param_tl = cv2.SimpleBlobDetector_Params()

        # Assign values to object variables
        for key, val in self.parameters['blob_detector_db'].items():
            setattr(bd_param_db, key, val)
        for key, val in self.parameters['blob_detector_tl'].items():
            setattr(bd_param_tl, key, val)

        # Create a detector with the parameters
        self.detector_car = cv2.SimpleBlobDetector_create(bd_param_db)
        self.detector_tl = cv2.SimpleBlobDetector_create(bd_param_tl)

        # Publishers
        self.pub_raw_detections = rospy.Publisher("~raw_led_detection", LEDDetectionArray,queue_size=1)
        self.pub_image_right = rospy.Publisher("~image_detection_right", Image, queue_size=1)
        self.pub_image_front = rospy.Publisher("~image_detection_front", Image, queue_size=1)
        self.pub_image_TL = rospy.Publisher("~image_detection_TL", Image, queue_size=1)
        self.pub_detections = rospy.Publisher("~led_detection", SignalsDetection, queue_size=1)
        self.pub_debug = rospy.Publisher("~debug_info", LEDDetectionDebugInfo, queue_size=1)

        # Subscribers
        self.sub_cam = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.sub_trig = rospy.Subscriber("~trigger", Byte, self.trigger_callback)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)

        # Additional parameters
        self.protocol = rospy.get_param("~LED_protocol")

        # Detect continuously as long as active [INTERACTIVE MODE] set to False for manual trigger
        self.continuous = rospy.get_param('~continuous', False)

        # Cell size (needed for visualization)
        self.cell_size = rospy.get_param("~cell_size")
        self.crop_rect_norm = rospy.get_param("~crop_rect_normalized")

        # Get frequency to identify
        self.freqIdentify = self.protocol['frequencies'].values()

        # Check vehicle name
        self.veh_name = rospy.get_namespace().strip("/")
        if not self.veh_name:
            # fall back on private param passed thru rosrun
            # syntax is: rosrun <pkg> <node> _veh:=<bot-id>
            if rospy.has_param('~veh'):
                self.veh_name = rospy.get_param('~veh')

            else:
                raise ValueError('Vehicle name is not set.')

        # Log info
        self.log('[%s] Vehicle: %s' % (self.node_name, self.veh_name))
        self.log('[%s] Waiting for camera image...' % self.node_name)

    def cbSwitch(self, switch_msg):
        """Callback that turns on/off the node.

            Reads the switch from the Finite State Machine and sets
            self.active accordingly.

            Args:
                switch_msg (BoolStamped): Switch for the node.
        """
        self.active = switch_msg.data

    def camera_callback(self, msg):
        if not self.active:
            return

        float_time = msg.header.stamp.to_sec()
        debug_msg = LEDDetectionDebugInfo()

        if self.trigger:
            # self.log('[%s] GOT TRIGGER! Starting...')
            self.trigger = False
            self.data = []
            self.capture_finished = False
            # Start capturing images
            # self.log('[%s] Start capturing frames'%self.node_name)
            self.first_timestamp = msg.header.stamp.to_sec()
            self.t_init = time.time()

        elif self.capture_finished:
            self.node_state = 0
            # self.log('[%s] Waiting for trigger...' %self.node_name)

        if self.first_timestamp > 0:
            # TODO sanity check rel_time positive, restart otherwise
            rel_time = float_time - self.first_timestamp

            # Capturing
            if rel_time < self.capture_time:
                self.node_state = 1
                # Capture image
                rgb = numpy_from_ros_compressed(msg)
                rgb = cv2.cvtColor(rgb, cv2.COLOR_BGRA2GRAY)
                rgb = cv2.resize(rgb, (640 * 1, 480 * 1))
                rgb = 255 - rgb
                # self.log('[%s] Capturing frame %s' %(self.node_name, rel_time))
                # Save image to data
                # if np.size(self.data) == 0:
                #    self.data = rgb
                # else:
                #    self.data = np.dstack((self.data,rgb))
                self.data.append({'timestamp': float_time, 'rgb': rgb[:, :]})
                debug_msg.capture_progress = 100.0 * rel_time / self.capture_time

            # Start processing
            elif not self.capture_finished and self.first_timestamp > 0:
                # self.log('[%s] Relative Time %s, processing' %(self.node_name, rel_time))
                self.node_state = 2
                self.capture_finished = True
                self.first_timestamp = 0

                # IMPORTANT! Explicitly ignore messages while processing, accumulates delay otherwise!
                self.sub_cam.unregister()

                self.send_state(debug_msg)
                # Process image and publish results
                self.process_and_publish()

        self.send_state(debug_msg)  # TODO move heartbeat to dedicated thread

    def trigger_callback(self, msg):
        self.trigger = True

    def crop_image(self, images, crop_norm):
        # Get size
        height, width, = images.shape
        # Compute indices
        h_start = int(np.floor(height * crop_norm[0][0]))
        h_end = int(np.ceil(height * crop_norm[0][1]))
        w_start = int(np.floor(width * crop_norm[1][0]))
        w_end = int(np.ceil(width * crop_norm[1][1]))
        # Crop image
        image_cropped = images[h_start:h_end, w_start:w_end, :]
        # Return cropped image
        return image_cropped

    def process_and_publish(self):
        # Initial time
        tic = time.time()

        # Get dimensions
        h, w = self.data[0]['rgb'].shape
        num_img = len(self.data)

        # Save in proper vectors
        images = np.zeros((h, w, num_img), dtype=np.uint8)
        timestamps = np.zeros(num_img)
        for i, v in enumerate(self.data):
            timestamps[i] = v['timestamp']
            images[:, :, i] = v['rgb']

        # Crop images
        img_right = self.crop_image(images, self.parameters['crop_params']['cropNormalizedRight'])
        img_front = self.crop_image(images, self.parameters['crop_params']['cropNormalizedFront'])
        img_tl = self.crop_image(images, self.parameters['crop_params']['cropNormalizedTL'])

        # Print on screen
        # self.log('[%s] Analyzing %s images of size %s X %s' %(self.node_name,num_img,w,h))

        # Get blobs right
        blobs_right, frame_right = self.detect_blobs(img_right)
        # Get blobs front
        blobs_front, frame_front = self.detect_blobs(img_front)
        # Get blobs right
        blobs_tl, frame_tl = self.detect_blobs(img_tl)

        radius = self.DTOL/2.0
        
        # Extract blobs
        keypoint_blob_right = self.extract_blobs(blobs_right, radius)
        keypoint_blob_front = self.extract_blobs(blobs_front, radius)
        keypoint_blob_tl = self.extract_blobs(blobs_tl, radius)

        # Images
        img_pub_right = cv2.drawKeypoints(img_right[:, :, -1], keypoint_blob_right, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        img_pub_front = cv2.drawKeypoints(img_front[:, :, -1], keypoint_blob_front, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        img_pub_tl = cv2.drawKeypoints(img_tl[:, :, -1], keypoint_blob_tl, np.array([]), (0, 0, 255),
                                       cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Initialize detection
        self.right = SignalsDetection.NO_CAR
        self.front = SignalsDetection.NO_CAR
        self.traffic_light = SignalsDetection.NO_TRAFFIC_LIGHT

        # Result
        result = LEDDetectionArray()

        # Sampling time
        t_s = (1.0*self.capture_time)/(1.0*num_img)

        # Decide whether LED or not
        self.right = self.is_led(blobs_right, t_s, num_img, h, w,
                                 self.parameters['~crop_params']['cropNormalizedRight'], timestamps, result)
        self.front = self.is_led(blobs_right, t_s, num_img, h, w,
                                 self.parameters['~crop_params']['cropNormalizedFront'], timestamps, result)
        self.traffic_light = self.is_led(blobs_right, t_s, num_img, h, w,
                                         self.parameters['~crop_params']['cropNormalizedTL'], timestamps, result)

        # Left bot (also UNKNOWN)
        self.left = "UNKNOWN"

        # Final time
        processing_time = time.time()-tic
        total_time = time.time() - self.t_init

        # Publish results
        self.publish(img_pub_right, img_pub_front, img_pub_tl, result)

        # Print performance
        # self.log('[%s] Detection completed. Processing time: %.2f s. Total time:  %.2f s' %(self.node_name,
        #                                                                                    processing_time,
        #                                                                                    total_time))

        # Keep going
        if self.continuous:
            self.trigger = True
            self.sub_cam = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.camera_callback)

    def get_blobs(self, images):

        blobs = []
        frame = []
        # Iterate over time
        num_images = len(images)
        for t in range(num_images):
            keypoints = self.detector.detect(images[:, :, t])
            frame.append(np.zeros((2, len(keypoints))))

            for n in range(len(keypoints)):
                frame[t][:, n] = keypoints[n].pt
                if len(blobs) == 0:
                    # If no blobs saved, then save the first LED detected
                    blobs.append({'p': frame[t][:, n], 'N': 1, 'Signal': np.zeros(images.shape[2])})
                    blobs[-1]['Signal'][t] = 1
                else:
                    # Thereafter, check whether the detected LED belongs to a blob
                    dist = np.empty(len(blobs))
                    for k in range(len(blobs)):
                        dist[k] = np.linalg.norm(blobs[k]['p'] - blobs[t][:, n])
                    if np.min(dist) < self.DTOL:
                        if blobs[np.argmin(dist)]['Signal'][t] == 0:
                            blobs[np.argmin(dist)]['N'] += 1
                            blobs[np.argmin(dist)]['Signal'][t] = 1
                    else:
                        blobs.append({'p': frame[t][:, n], 'N': 1, 'Signal': np.zeros(images.shape[2])})
                        blobs[-1]['Signal'][t] = 1

        return blobs, frame

    def extract_blobs(self, blobs, radius):
        # Extract blobs
        keypoint_blob = []
        for k in range(len(blobs)):
            assert np.sum(blobs[k]['Signal']) == blobs[k]['N']
            keypoint_blob.append(cv2.KeyPoint(blobs[k]['p'][0], blobs[k]['p'][1], radius))
        return keypoint_blob

    def is_led(self, blobs, t_s, num_img, h, w, cropping, timestamps, result):
        # Decide whether LED or not (right)
        for i in range(len(blobs)):
            # self.log('[%s] Detection on the right' % (self.node_name))
            # Detection
            detected, result, freq_identified, fft_peak_freq = self.detect_blob(blobs[i], t_s, num_img, h, w,
                                                                                cropping, timestamps, result)
            # Take decision
            if detected:
                # print '-------------------'
                # print("num_img = %d " % num_img)
                # print("t_samp = %f " % t_samp)
                # print("fft_peak_freq = %f " % fft_peak_freq)
                # print("freq_identified = %f " % freq_identified)
                # print '-------------------'

                if freq_identified == self.freqIdentify[3]:
                    detected = SignalsDetection.SIGNAL_PRIORITY
                elif freq_identified == self.freqIdentify[4]:
                    detected = SignalsDetection.SIGNAL_SACRIFICE_FOR_PRIORITY
                else:
                    detected = SignalsDetection.SIGNAL_A

                return detected

    def detect_blob(self, blob, t_s, num_img, h, w, crop, timestamps, result):
        # Percentage of appearance
        appearance_percentage = (1.0*blob['N'])/(1.0*num_img)

        # Frequency estimation based on FFT
        f = np.arange(0.0, 1.0*num_img+1.0, 2.0)
        signal_f = scipy.fftpack.fft(blob['Signal']-np.mean(blob['Signal']))
        y_f = 2.0/num_img*np.abs(signal_f[:num_img/2+1])
        fft_peak_freq = 1.0*np.argmax(y_f)/(num_img*t_s)
        # half_freq_dist = 0.8 #1.0*f[1]/2

        # self.log('[%s] Appearance perceived. = %s, frequency = %s' % 
        #          (self.node_name, appearance_percentage, fft_peak_freq))
        freq_identified = 0
        # Take decision
        detected = False
        for i in range(len(self.freqIdentify)):
            if ((0.8 > appearance_percentage > 0.2 and not self.useFFT) or
                    (self.useFFT and abs(fft_peak_freq - self.freqIdentify[i]) < 0.35)):
                # Decision
                detected = True
                freq_identified = self.freqIdentify[i]
                # Raw detection
                coord_norm = Vector2D(1.0*(crop[1][0]+blob['p'][0])/w, 1.0*(crop[0][0]+blob['p'][1])/h)
                result.detections.append(LEDDetection(rospy.Time.from_sec(timestamps[0]),
                                                      rospy.Time.from_sec(timestamps[-1]),
                                                      coord_norm, fft_peak_freq, '', -1, timestamps, signal_f, f, y_f))

        return detected, result, freq_identified, fft_peak_freq

    def publish(self, img_right, img_front, img_tl, results):
        #  Publish image with circles
        img_right_circle_msg = self.bridge.cv2_to_imgmsg(img_right, encoding="passthrough")
        img_front_circle_msg = self.bridge.cv2_to_imgmsg(img_front, encoding="passthrough")
        img_tl_circle_msg = self.bridge.cv2_to_imgmsg(img_tl, encoding="passthrough")

        # Publish image
        self.pub_image_right.publish(img_right_circle_msg)
        self.pub_image_front.publish(img_front_circle_msg)
        self.pub_image_TL.publish(img_tl_circle_msg)

        # Publish results
        self.pub_raw_detections.publish(results)

        # Publish debug
        debug_msg = LEDDetectionDebugInfo()
        debug_msg.cell_size = self.cell_size
        debug_msg.crop_rect_norm = self.crop_rect_norm
        debug_msg.led_all_unfiltered = results
        debug_msg.state = 0
        self.pub_debug.publish(debug_msg)

        # Log info (right)
        if self.right != SignalsDetection.NO_CAR:
            self.log('Right: LED detected')
        else:
            self.log('Right: No LED detected')

        # Log info (front)
        if self.front != SignalsDetection.NO_CAR:
            self.log('Front: LED detected')
        else:
            self.log('Front: No LED detected')

        # Log info (TL)
        if self.traffic_light == SignalsDetection.STOP:
            self.log('[%s] Traffic Light: red' % self.node_name)
        elif self.traffic_light == SignalsDetection.GO:
            self.log('[%s] Traffic Light: green' % self.node_name)
        else:
            self.log('[%s] No traffic light' % self.node_name)

        # Publish
        self.log("[%s] The observed LEDs are:\n Front = %s\n Right = %s\n Traffic light state = %s" %
                 (self.node_name, self.front, self.right, self.traffic_light))
        self.pub_detections.publish(SignalsDetection(front=self.front,
                                                     right=self.right,
                                                     left=self.left,
                                                     traffic_light_state=self.traffic_light))

    def send_state(self, msg):
        msg.state = self.node_state
        self.pub_debug.publish(msg)


if __name__ == '__main__':
    rospy.spin()
    # Initialize the node
    camera_node = LEDDetectorNode(node_name='led_detector_node')
    # Keep it spinning to keep the node alive
    rospy.spin()

