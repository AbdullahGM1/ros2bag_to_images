import rclpy
import rosbag2_py
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Extract images from a ROS2 bag file.")
    parser.add_argument("bag_path", type=str, help="Path to the ROS2 bag file")
    parser.add_argument("--topic", type=str, default="/depth_map", help="Topic name to extract images from")
    parser.add_argument("--output_dir", type=str, default="depth_map_images", help="Directory to save images")
    args = parser.parse_args()

    # Initialize the ROS client library
    rclpy.init()

    # Directory where images will be saved
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    # Bridge to convert ROS image messages to OpenCV images
    bridge = CvBridge()

    # Open the bag file
    bag_path = args.bag_path
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")

    # Create reader
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get the topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # Set filter to only read from the specified topic by checking within the loop
    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        
        # Check if the message is from the specified topic
        if topic == args.topic:
            # Deserialize data into an Image message using rclpy serialization
            msg_type = get_message(type_map[topic])
            image_msg = deserialize_message(data, msg_type)
            
            # Convert the ROS image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            
            # Save the image
            image_path = os.path.join(output_dir, f"{count:04d}.png")
            cv2.imwrite(image_path, cv_image)
            
            print(f"Saved image {count} at {image_path}")
            count += 1

    # Close the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
