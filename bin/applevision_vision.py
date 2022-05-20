#!/usr/bin/python3

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge
from applevision_rospkg.msg import RegionOfInterestWithCovarianceStamped
from helpers import HeaderCalc
from applevision_vision import MODEL_PATH

CONFIDENCE_THRESH = 0.5


def format_yolov5(
    source
):  #Function taken from medium: https://medium.com/mlearning-ai/detecting-objects-with-yolov5-opencv-python-and-c-c7cf13d1483c
    # YOLOV5 needs a square image. Basically, expanding the image to a square
    # put the image in square big enough
    col, row, _ = source.shape
    _max = max(col, row)
    resized = np.zeros((_max, _max, 3), np.uint8)
    resized[0:col, 0:row] = source
    # resize to 640x640, normalize to [0,1[ and swap Red and Blue channels
    result = cv2.dnn.blobFromImage(resized, 1 / 255.0, (640, 640), swapRB=True)
    return result


def unwrap_detection(input_image, output_data):
    class_ids = []
    confidences = []
    boxes = []

    rows = output_data.shape[0]
    image_width, image_height, _ = input_image.shape

    x_factor = image_width / 360
    y_factor = image_height / 640

    for r in range(rows):
        row = output_data[r]
        confidence = row[4]
        if confidence >= CONFIDENCE_THRESH:

            classes_scores = row[5:]
            _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):

                confidences.append(confidence)
                class_ids.append(class_id)

                x, y, w, h = row[0].item(), row[1].item(), row[2].item(
                ), row[3].item()
                left = int((x - 0.5 * w) * x_factor)
                top = int((y - 0.5 * h) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                box = np.array([left, top, width, height])
                boxes.append(box)
    return class_ids, confidences, boxes


class AppleVisionHandler:

    def __init__(self, net, topic: str, frame: str,
                 debug_frame_topic: str) -> None:
        self.net = net
        self.pub = rospy.Publisher(topic,
                                   RegionOfInterestWithCovarianceStamped,
                                   queue_size=10)
        self.pub_debug = rospy.Publisher(debug_frame_topic,
                                         Image,
                                         queue_size=10)
        self._br = CvBridge()
        self._header = HeaderCalc(frame)

    def run_applevision(self, im: Image):
        if rospy.Time.now() - im.header.stamp > rospy.Duration.from_sec(0.1):
            # this frame is too old
            rospy.logwarn_throttle_identical(3, 'CV: Ignoring old frame')
            return None

        # convert to cv2 image
        start = time.time()
        frame = self._br.imgmsg_to_cv2(im, 'bgr8')
        adjusted_image = format_yolov5(frame)
        self.net.setInput(adjusted_image)
        predictions = self.net.forward()
        output = predictions[0]
        class_ids, confidences, boxes = unwrap_detection(frame, output)

        # Remove duplicates using non-max suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESH, 0.45)
        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        # print(confidences, boxes, "\n")
        if len(confidences) > 0:
            # Order them for fast indexing later
            boxes = [tuple(b) for b in boxes]
            confidences, boxes = zip(
                *sorted(zip(confidences, boxes), reverse=True))

            # TODO: base varience off of confidence
            msg = RegionOfInterestWithCovarianceStamped(
                header=self._header.get_header(),
                x=boxes[0][0],
                y=boxes[0][1],
                w=boxes[0][2],
                h=boxes[0][3],
                x_var=30**2,
                y_var=30**2,
                w_var=30**2,
                h_var=30**2)
            self.pub.publish(msg)

            # attempted to put in json format string
            # string_to_write = "{"+"confidence: {}".format(confidences[0]) + ", box (x,y,w,h): {}".format(boxes[0])+ "}"
            # write_coords(string_to_write)

            # Display the resulting frame
            box = boxes[0]
            conf = confidences[0]
            color = (255, 255, 0)
            cv2.rectangle(frame, box, color, 2)
            cv2.rectangle(frame, (box[0], box[1] - 20),
                          (box[0] + box[2], box[1]), color, -1)
            cv2.putText(frame, "Apple", (box[0] + 5, box[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        else:
            msg = RegionOfInterestWithCovarianceStamped(
                header=self._header.get_header(),
                x=0,
                y=0,
                w=0,
                h=0,
                x_var=np.inf,
                y_var=np.inf,
                w_var=np.inf,
                h_var=np.inf)
            self.pub.publish(msg)

        debug_im = self._br.cv2_to_imgmsg(frame)
        self.pub_debug.publish(debug_im)

        end = time.time()
        print(f'Took {end - start:.2f} secs')


if __name__ == '__main__':
    rospy.init_node('applevision_fake_sensor_data')

    net = cv2.dnn.readNet(str(MODEL_PATH))
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

    # TODO: add image_proc
    vision_handler = AppleVisionHandler(net, 'applevision/apple_camera',
                                        'palm_camera',
                                        'applevision/debug_apple_camera')
    sub = rospy.Subscriber('palm_camera/image_rect_color',
                           Image,
                           vision_handler.run_applevision,
                           queue_size=20)

    rospy.spin()
