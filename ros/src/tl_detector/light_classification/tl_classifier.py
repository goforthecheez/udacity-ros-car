from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import time
import rospy

class TLClassifier(object):
    def __init__(self, is_site):
        
        if is_site:
            # ssd_inception_v2_coco_2017_11_17 Model with 20000 steps on Udacity site data
            PATH_TO_GRAPH = r'light_classification/models/ssd_i_coco/real/frozen_inference_graph.pb'
            # faster_rcnn_inception_v2_coco_2018_01_28 Model with 10000 steps on Udacity site data
            # More accurate than ssd inception but slower - choose ssd inception is enough for Udacity site.
            #PATH_TO_GRAPH = r'light_classification/models/frcnn/frozen_inference_graph.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/models/ssd_i_coco/sim/frozen_inference_graph.pb'

        self.graph = tf.Graph()
        self.threshold = 0.3

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
        start = time.time()
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
        
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        #rospy.logwarn("classes: {}".format(' '.join([str(c) for c in classes])))
        #rospy.logwarn("scores: {}".format(' '.join([str(s) for s in scores])))

        colors = [TrafficLight.GREEN, TrafficLight.RED, TrafficLight.YELLOW]
        votes = [0.0] * 3
        for i, s in enumerate(scores):
            if s > self.threshold:
               if classes[i] == 1:  # green
                   votes[0] = votes[0] + s
               elif classes[i] == 2:  # red
                   votes[1] = votes[1] + s
               elif classes[i] == 3:  # yellow
                   votes[2] = votes[2] + s
        max_idx = np.argmax(votes)
        color = colors[max_idx]

        #rospy.logwarn("votes: green={}, red={}, yellow={}".format(votes[0], votes[1], votes[2]))

        elapsed = time.time() - start
        rospy.logwarn("Traffic light color is {} | Time elapsed: {} sec".format(color, elapsed))
        return color

