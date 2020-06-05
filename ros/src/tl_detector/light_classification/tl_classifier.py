import numpy as np
import tensorflow as tf
import rospy

from .object_detection.utils import label_map_util

from styx_msgs.msg import TrafficLight
from std_msgs.msg import String
from scipy.stats import mode

PATH_TO_FROZEN_GRAPH = 'light_classification/ssd_custom_graph/frozen_inference_graph_tf_1.3.pb'
PATH_TO_LABELS = 'light_classification/label_map.pbtxt'
NUM_CLASSES = 3
SCORE_THRESH = 0.80
class_lookup = {
        1 : TrafficLight.GREEN,
        2 : TrafficLight.YELLOW,
        3 : TrafficLight.RED,
}


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.detection_graph, self.label_map, self.categories, self.category_index = self.import_graph()
        self.tf_config = tf.ConfigProto()
        self.tf_config.gpu_options.allow_growth = True
        self.sess = tf.Session(graph=self.detection_graph, config=self.tf_config)

    def import_graph(self):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
        return detection_graph, label_map, categories, category_index

    def inference(self, image):
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        image_np_expanded = np.expand_dims(image, axis=0)

        (boxes, scores, classes, num) = self.sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        return boxes, scores, classes, num

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        boxes, scores, classes, num = self.inference(image)
        scores = scores[0]
        classes = classes[0]
        good_scores = np.argwhere(scores > SCORE_THRESH)
        good_classes = classes[good_scores]
        if len(good_scores) < 1:
            # No detections
            return TrafficLight.UNKNOWN
        class_mode = int(mode(good_classes)[0][0][0])
        return class_lookup[class_mode]

