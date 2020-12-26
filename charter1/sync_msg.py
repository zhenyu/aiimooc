
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2

_input_lidar_topic ="/rslidar_points"
_input_image_topic = "/zed/zed_node/left/image_rect_color"

_output_lidar_topic= "/zhenyusha_point_cloud"
_output_image_topic = "/zhenyusha_img"

def callback(input_image, input_point_cloud):
    rospy.loginfo("incallback")
    
if __name__ == '__main__':
   rospy.init_node('sync_msg_time', anonymous=False) 
   image_sub = message_filters.Subscriber(_input_image_topic, Image)
   point_cloud_sub = message_filters.Subscriber(_input_lidar_topic, PointCloud2) 
   ts = message_filters.TimeSynchronizer([image_sub, point_cloud_sub], 10)
   ts.registerCallback(callback)
   rospy.spin()