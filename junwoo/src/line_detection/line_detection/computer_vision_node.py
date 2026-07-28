import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from .lane_detector import LaneDetector
class ComputerVisionNode(Node):
 def __init__(self):
  super().__init__('computer_vision_node'); d={'image_topic':'/camera/image_raw','src_pts':[.40,.55,.60,.55,.99,.98,.01,.98],'dst_pts':[.18,.0,.82,.0,.82,1.,.18,1.],'white_hls_low':[0,120,0],'white_hls_high':[180,255,100],'yellow_hsv_low':[12,35,55],'yellow_hsv_high':[45,255,255],'min_pixels':25,'lookahead_ratio':.65,'offset_gain':.7,'heading_gain':1.,'canny_low':45,'canny_high':130,'min_component_area':40,'min_component_aspect':1.35,'fit_residual_px':24.,'temporal_alpha':.35}; [self.declare_parameter(k,v) for k,v in d.items()]; p={k:self.get_parameter(k).value for k in d}; self.d=LaneDetector(p); self.b=CvBridge(); self.create_subscription(Image,p['image_topic'],self.cb,10); self.e=self.create_publisher(Float32,'/lane_detection/steering_error',10); self.o=self.create_publisher(Float32,'/lane_detection/lateral_error',10); self.h=self.create_publisher(Float32,'/lane_detection/heading_angle',10); self.v=self.create_publisher(Bool,'/lane_detection/lane_valid',10); self.c=self.create_publisher(Float32,'/lane_detection/lane_confidence',10); self.db=self.create_publisher(Image,'/lane_detection/debug_image',10); self.m=self.create_publisher(Image,'/lane_detection/binary_mask',10)
 def cb(self,msg):
  try:o,h,e,dbg,mask,valid,confidence=self.d.detect(self.b.imgmsg_to_cv2(msg,'bgr8'))
  except Exception as x:self.get_logger().error(str(x));return
  for pub,v in ((self.e,e),(self.o,o),(self.h,h)): z=Float32();z.data=v;pub.publish(z)
  z=Bool();z.data=valid;self.v.publish(z); z=Float32();z.data=confidence;self.c.publish(z)
  for pub,im,enc in ((self.db,dbg,'bgr8'),(self.m,mask,'mono8')): z=self.b.cv2_to_imgmsg(im,enc);z.header=msg.header;pub.publish(z)
def main():
 rclpy.init();n=ComputerVisionNode();rclpy.spin(n)
