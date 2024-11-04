import rclpy
import rosbag2_py
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    # Initialize the ROS client library
    rclpy.init()

    # Directory where images will be saved
    output_dir = "depth_map_images" 
    os.makedirs(output_dir, exist_ok=True)

    # Bridge to convert ROS image messages to OpenCV images
    bridge = CvBridge()

    # Open the bag file
    bag_path = "/home/user/shared_volume/my_depth_map_bag/aa.db3"  # Updated bag file path
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")

    # Create reader
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get the topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Set filter to only read from /depth_map topic by checking within the loop
    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        
        # Check if the message is from the /depth_map topic
        if topic == "/depth_map":
            # Deserialize data into an Image message using rclpy serialization
            msg_type = get_message(type_map[topic])
            image_msg = deserialize_message(data, msg_type)
            
            # Convert the ROS image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            
            # Save the image
            image_path = os.path.join(output_dir, f"depth_map_{count:04d}.png")
            cv2.imwrite(image_path, cv_image)
            
            print(f"Saved image {count} at {image_path}")
            count += 1

    # Close the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
