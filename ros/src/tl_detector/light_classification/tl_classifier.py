from styx_msgs.msg import TrafficLight
import numpy as np
import rospkg
import os
import rospy
import tensorflow as tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.color_th = 15

        
        rospack    = rospkg.RosPack()
        model_dir  = os.path.join(rospack.get_path('tl_detector/light_classification'), 'model')
        model_path = os.path.join(model_dir, 'frozen_inference_graph.pb')

        self.config = tf.ConfigProto()
        # load object detection graph
        self.graph  = self.load_graph(model_path, self.config)
        # tensorflow session
        self.sess   = tf.Session(graph=self.graph, config=self.config)
        # definition of input and output tensors for the detection graph.
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        # each box represents a part of the image in which a specific object has been located.
        self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        # for each detected object the score represents the confidence level.
        self.scores = self.graph.get_tensor_by_name('detection_scores:0')
        # this is the MS COCO dataset class, we only need class 10 for the traffic light.
        self.classes = self.graph.get_tensor_by_name('detection_classes:0')
        # minimum required confidence for traffic lights
        self.confidence_th = 0.1

        # Traffic light publisher.
        self.bridge = CvBridge()
        self.tl_image = rospy.Publisher('/tl_detector/traffic_light', Image, queue_size=1)

        rospy.loginfo('tl_classifier init complete')

    # Function to load a graph from a proto buf file.
    def load_graph(self, model, config):
        with tf.Session(graph = tf.Graph(), config=config) as sess:
            assert tf.get_default_session() is sess

            gd = tf.GraphDef()
            with tf.gfile.Open(model, 'rb') as f:
                data = f.read()
                gd.ParseFromString(data)

            tf.import_graph_def(gd, name='')
            graph = tf.get_default_graph()
            #print('Graph v' + str(graph.version) + ', nodes: ' + ', '.join([n.name for n in graph.as_graph_def().node]))

            return graph

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return self.predict_detection(image)

    def predict_detection(self, image):

        h, w, _ = image.shape
        image_exp = cv2.resize(image, (300, 300))
        image_exp = np.expand_dims(image, axis=0)

        boxes, scores, classes = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={self.image_tensor: image_exp}
            )

        # post processing
        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        # Get the box with the highest score.
        conf_max = 0
        idx = -1
        for i in range(boxes.shape[0]):
            if scores[i] > self.confidence_th and classes[i] == 10:
                if scores[i] > conf_max:
                    conf_max = scores[i]
                    idx = i

        prediction = TrafficLight.UNKNOWN
        # if there is a traffic light with high confidence (max confidence traffic light is selected)
        if idx != -1:
            # create a tuple for the box
            box = tuple(boxes[idx].tolist())

            # corners of the box
            top, left, bottom, right = box

            # resize and expand a little more
            left   = int(max(0, (left * w) - 3))
            top    = int(max(0, (top * h) - 5))
            bottom = int(min(h, (bottom * h) + 5))
            right  = int(min(w, (right * w) + 3))

            box = left, top, right, bottom

            ROI = image[top:bottom, left:right]
            tl_image = cv2.resize(ROI, (32, 32))
            self.tl_image.publish(self.bridge.cv2_to_imgmsg(tl_image, "rgb8"))
            
            #rospy.loginfo(ROI.shape)

            #rospy.loginfo(box)

            prediction = self.predict_color(ROI)

        else:
            # publish empty image
            self.tl_image.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.zeros((32, 32), np.uint8), cv2.COLOR_GRAY2RGB), "rgb8"))


        #rospy.loginfo(prediction)
        return prediction


    def predict_color(self, image):
        """
        image: cv2.Image (RGB)
        """
        R = image[:,:,0]
        G = image[:,:,1]
        R_area = np.sum(R > R.max() - 5)
        G_area = np.sum(G > G.max() - 5)
        #rospy.loginfo(G_area)

        prediction = TrafficLight.UNKNOWN

        if R_area >= self.color_th and G_area <= self.color_th:
            prediction = TrafficLight.RED
        elif R_area >= self.color_th and G_area >= self.color_th:
            prediction = TrafficLight.YELLOW
        elif G_area >= self.color_th:
            prediction = TrafficLight.GREEN
        else:
         prediction = TrafficLight.UNKNOWN

        if prediction == TrafficLight.RED:
            print("red")
        elif prediction == TrafficLight.YELLOW:
            print("yellow")
        elif prediction == TrafficLight.GREEN:
            print("green")
        else:
            print("unknown - clear")
        
        return prediction