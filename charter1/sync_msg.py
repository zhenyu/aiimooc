import rospy
import rosbag
import message_filters
from sensor_msgs.msg import Image, PointCloud2

_input_lidar_topic = "/rslidar_points"
_input_image_topic = "/zed/zed_node/left/image_rect_color"

_output_lidar_topic = "/zhenyusha_point_cloud"
_output_image_topic = "/zhenyusha_img"
#output bag file
_bag = None


def callback(input_image, input_point_cloud):
    _bag.write(topic=_output_lidar_topic,
               msg=input_point_cloud,
               t=input_point_cloud.header.stamp)
    _bag.write(topic=_output_image_topic,
               msg=input_image,
               t=input_point_cloud.header.stamp)


if __name__ == '__main__':
    try:
        rospy.init_node('sync_msg_time', anonymous=False)
        #TODO file name args
        _bag = rosbag.Bag('charpter1.bag', 'w')

        image_sub = message_filters.Subscriber(_input_image_topic, Image)
        point_cloud_sub = message_filters.Subscriber(_input_lidar_topic,
                                                     PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, point_cloud_sub], 10, 0.1, allow_headerless=False)
        ts.registerCallback(cb=callback)
        rospy.spin()
    finally:
        _bag.close()
