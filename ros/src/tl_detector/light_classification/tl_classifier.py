from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self, is_site):
        # The model testing result is on https://github.com/HiepTang/capstone-traffic-light/blob/master/tl_classification.ipynb
        # The thresold will set following this result
        if is_site:
            # ssd_inception_v2_coco_2017_11_17 model - the threshold only about 0.60
            PATH_TO_GRAPH = r'light_classification/models/ssd_i_coco/real/frozen_inference_graph.pb'
            self.threshold = .6
            # faster_rcnn_inception_v2_coco_2018_01_28 - the threshold can set to .8 or .9
            # Consider about performance
            # Uncomment these lines and comment out ssd lines if you want to use frcnn model.
            # PATH_TO_GRAPH = r'light_classification/models/frcnn/frozen_inference_graph.pb'
            # self.threshold = .8
        else:
            PATH_TO_GRAPH = r'light_classification/models/ssd_i_coco/sim/frozen_inference_graph.pb'
            self.threshold = .65

        self.graph = tf.Graph()
        
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')

        self.sess = tf.Session(graph=self.graph)
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = rospy.get_rostime().secs
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = rospy.get_rostime().secs
            rospy.logdebug('classification duration {} seconds'.format(end - start))
        
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        
        if scores[0] > self.threshold:
            if classes[0] == 1:
                return TrafficLight.GREEN
            elif classes[0] == 2:
                return TrafficLight.RED
            elif classes[0] == 3:
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
