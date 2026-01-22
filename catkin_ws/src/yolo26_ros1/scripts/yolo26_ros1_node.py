#!/opt/venv/bin/python
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class Yolo26Ros1Node:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image", "/camera/rgb/image_raw")
        self.out_topic = rospy.get_param("~out_image", "/yolo/image")
        self.model_path = rospy.get_param("~model", os.environ.get("YOLO_MODEL", "yolo26n.pt"))
        self.conf = float(rospy.get_param("~conf", os.environ.get("YOLO_CONF", "0.25")))
        self.device = rospy.get_param("~device", os.environ.get("YOLO_DEVICE", ""))  # ""=auto

        rospy.loginfo(f"[yolo] model={self.model_path} conf={self.conf} device={self.device}")
        rospy.loginfo(f"[yolo] sub={self.image_topic} pub={self.out_topic}")

        # NOTE: If model_path is a name (e.g., yolo26n.pt), ultralytics may download it into its cache.
        self.model = YOLO(self.model_path)

        self.pub = rospy.Publisher(self.out_topic, Image, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

    def cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            res = self.model.predict(source=bgr, conf=self.conf, device=self.device, verbose=False)[0]
            vis = res.plot()  # BGR annotated image
            out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            out.header = msg.header
            self.pub.publish(out)
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[yolo] error: {e}")


if __name__ == "__main__":
    rospy.init_node("yolo26_ros1_node")
    Yolo26Ros1Node()
    rospy.spin()

