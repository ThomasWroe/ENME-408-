#!/usr/bin/env python3
# Thomas Wroe
# Description: Record the position and quaternion of the end effector when 'a' is pressed.
# Ends recording when 'q' is pressed. Then moves the gripper to the start position and rotates joint 5.
# Additionally, detect Hough circles and ArUco markers from the wrist camera.

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
import intera_external_devices
from geometry_msgs.msg import Pose, Point, Quaternion
import math

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
from collections import deque  # Import deque for buffering

# Initialize global variables
limb = None
pose_variables = {}
counter = 0
recording_active = True 
pub = rospy.Publisher("beltCommand", Bool, queue_size=10)
global condition 
global color_name # Flag for ArUco marker detection
color_detected = False # Flag for color detection
frame_counter = 0 # Frame counter for color detection
pixel_buffer = deque(maxlen=10) # Buffer for pixel values
global message # Flag to control the recording loop


def detect_shapes_from_camera():
    """
    Stream from the wrist camera and detect circles and ArUco markers.
    Display the camera feed with the detections overlayed.
    Exits as soon as a shape or marker is detected and returns True.
    """
    try:
        # Initialize CvBridge for converting ROS images to OpenCV
        bridge = CvBridge()

        # Camera topic (adjust based on your setup if needed)
        camera_topic = "/io/internal_camera/right_hand_camera/image_rect"

        # Initialize the camera interface safely
        try:
            cameras = intera_interface.Cameras()
            if not cameras.verify_camera_exists("right_hand_camera"):
                rospy.logerr("Right hand camera does not exist.")
                return False
        except Exception as e:
            rospy.logerr(f"Failed to initialize camera interface: {e}")
            return False

        # Set camera gain and exposure
        try:
            # rospy.loginfo("Setting camera gain and exposure...")
            cameras.set_gain("right_hand_camera", 60)
            cameras.set_exposure("right_hand_camera", 5)
            rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr(f"Failed to set camera gain/exposure: {e}")
            return False

        # Start streaming
        try:
            # rospy.loginfo("Starting camera stream...")
            cameras.start_streaming("right_hand_camera")
        except Exception as e:
            rospy.logerr(f"Failed to start camera streaming: {e}")
            return False

        rospy.loginfo("Starting shape detection. Press 'q' to exit.")

        detection_flag = False  # Will become True if a shape or marker is detected
        frame_counter = 0  # Counter to track the number of frames processed

        def process_image(msg):
            nonlocal detection_flag, frame_counter
            global condition
            if detection_flag:  # Stop processing if detection is complete
                return

            try:
                frame_counter += 1

                # Skip the first 10 frames
                if frame_counter <= 10:
                    return

                # Convert ROS image to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Detect Hough circles
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                gray = cv2.medianBlur(gray, 5)

                circles = cv2.HoughCircles(
                    gray,
                    cv2.HOUGH_GRADIENT,
                    dp=1,
                    minDist=50,
                    param1=50,
                    param2=60,
                    minRadius=10,
                    maxRadius=100,
                )
                if circles is not None:
                    condition = "HCT"
                    detection_flag = True  # Set the flag to stop processing
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        rospy.loginfo(f"Circle detected at: x={x}, y={y}, radius={r}")
                        cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)
                        cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)

                # Detect ArUco markers
                aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                parameters = cv2.aruco.DetectorParameters_create()
                camera_matrix = np.array([[634.5274345, 0, 368.45124349],
                                          [0, 634.26405667, 231.47068418],
                                          [0, 0, 1]], dtype=float)
                dist_coeffs = np.array([[0.03966622, -0.1798598, -0.00476518, -0.00085606, 0.74990008]])

                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                if ids is not None and len(corners) > 0:
                    condition = "aruco"
                    detection_flag = True  # Set the flag to stop processing
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
                    for i in range(len(ids)):
                        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                        cv2.drawFrameAxes(cv_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)
                        rospy.loginfo(f"Marker ID: {ids[i]}, Rotation: {rvecs[i]}, Translation: {tvecs[i]}")

                # Display the camera feed
                cv2.imshow("Wrist Camera Feed", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    detection_flag = True

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
            except Exception as e:
                rospy.logerr(f"Error processing image: {e}")

        # Subscribe to the wrist camera topic
        rospy.Subscriber(camera_topic, Image, process_image)

        # Main loop for detection
        while not detection_flag and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Cleanup
        try:
            if cameras.is_streaming("right_hand_camera"):
                cameras.stop_streaming("right_hand_camera")
                # rospy.loginfo("Stopped camera stream.")
        except Exception as e:
            rospy.logwarn(f"Failed to stop camera stream: {e}")
        cv2.destroyAllWindows()
        # rospy.loginfo("Cleaned up OpenCV windows.")

        return condition, detection_flag

    except Exception as e:
        rospy.logerr(f"Error during shape detection: {e}")
        return False

def record_joint_angles():
    """
    Records 10 different joint angle sets and stores them in a dictionary.
    """
    limb = intera_interface.Limb('right')
    joint_angles_recorded = {}

    print("Starting joint angle recording...")
    for i in range(1, 13):
        input(f"Move the robot to desired position for Joint Set {i} and press Enter to record...")
        current_joint_angles = limb.joint_angles()
        joint_angles_recorded[f"Set_{i}"] = current_joint_angles
        # print(f"Joint Set {i} recorded: {current_joint_angles}")

    print("Finished recording all 10 joint angle sets.")
    return joint_angles_recorded

def move_to_angles(joint_angles_recorded, condition, color_name):
    """
    Moves the robot to one of the joint angle sets based on the provided condition and color.

    Args:
        joint_angles_recorded (dict): The dictionary containing recorded joint angle sets.
        condition (str): "aruco" or "HCT".
        color (str): The color condition (e.g., "blue", "red", "green", etc.).
    """
    conditions = condition
    color_names = color_name
    valid_conditions = [
        ("aruco", "blue"), ("aruco", "red"), ("aruco", "green"),
        ("aruco", "yellow"), ("aruco", "orange"), ("aruco", "purple"),
        ("HCT", "orange"), ("aruco", "unknown"), ("HCT", "unknown"), ("HCT", "unknown"),
        ("HCT", "yellow"), ("HCT", "blue"), ("HCT", "green"), ("HCT", "purple")
    ]

    if (condition, color_name) not in valid_conditions:
        print(f"Invalid condition and color combination: ({conditions}, {color_names}).")
        return

    # Map conditions and colors to specific joint sets
    index_map = {
        ("aruco", "blue"): "Set_1",
        ("aruco", "red"): "Set_2",
        ("aruco", "green"): "Set_3",
        ("aruco", "yellow"): "Set_4",
        ("aruco", "orange"): "Set_5",
        ("aruco", "purple"): "Set_6",
        ("HCT", "blue"): "Set_7",
        ("HCT", "green"): "Set_8",
        ("HCT", "purple"): "Set_9",
        ("HCT", "orange"): "Set_10",
        ("HCT", "yellow"): "Set_11",
        ("aruco", "unknown"): "Set_12",
        ("HCT", "unknown"): "Set_11",
        ("HCT", "unknown"): "Set_11"
    }

    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(0.2)
    set_key = index_map.get((condition, color_names))
    if set_key and set_key in joint_angles_recorded:
        target_joint_angles = joint_angles_recorded[set_key]
        print(f"Moving to joint angles: {target_joint_angles}")
        limb.move_to_joint_positions(target_joint_angles)
    else:
        print(f"No joint angles recorded for {set_key}. Please record first.")   


def move_to_joint_angles(joint_angles):
    """
    Moves Sawyer's arm to the specified joint angles.

    Args:
        joint_angles (dict): A dictionary where keys are joint names (e.g., 'right_j0') and values are joint angles in radians.
                             Example: {'right_j0': 0.5, 'right_j1': -0.5, 'right_j2': 1.0, ...}
    """
    # # Initialize a ROS node
    # rospy.init_node('move_to_joint_angles', anonymous=True)

    # Create an instance of the Limb class to control Sawyer's arm
    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(0.2)  # Set the speed for joint movement
    # Command the limb to move to the specified joint angles
    # print("Moving to joint angles:", joint_angles)
    limb.move_to_joint_positions(joint_angles)

    print("Movement complete.")


def camera_color_detector():
    """
    Detects the average RGB values of the center pixel and determines the color name.
    Returns the RGB values and the detected color name.
    """

    frame_counter = 0  # Reset frame counter for each image
    color_detected = False  # Reset color detection flag for each image
    pixel_buffer = deque(maxlen=10) 

    def get_color_name(r, g, b):
        """
        Determines the color name based on RGB values.
        """
        if 226 <= r <= 255 and 125 <=g <=180 and 130 <= b <= 178:
            return "orange"
        elif 125 <= r <= 155 and 230<= g <= 255 and 230 <= b <= 255:
            return "green"
        elif 233<= r <= 255 and 191 <= g <= 231  and 161 <= b <= 201:
            return "red"
        elif 67 <= r <= 130 and 130 <= g <= 190 and 200 <= b <= 250:
            return "blue"
        elif 255 >= r >= 238 and 255 >= g >= 235 and 255 >= b >= 235:
            return "yellow"
        elif 180<= r <= 220 and 180<= g <= 220 and 235 <= b <= 255:
            return "purple"
        else:
            return "unknown"

    # Initialize variables (reset states for each call)
      # Buffer explicitly reset at the start of every call
    
    bridge = CvBridge()
     # Reset detection flag for each invocation

    # Debugging log
    rospy.loginfo("Starting camera color detection...")

    # Initialize camera safely
    try:
        cameras = intera_interface.Cameras()
        if not cameras.verify_camera_exists('head_camera'):
            rospy.logerr("Head camera not found!")
            return None, "Camera Error"
        cameras.start_streaming('head_camera')
        cameras.set_gain('head_camera', 50)
        cameras.set_exposure('head_camera', 30)
    except Exception as e:
        rospy.logerr(f"Failed to initialize camera: {e}")
        return None, "Camera Initialization Failed"

    def process_image(msg):
        nonlocal frame_counter, color_detected, pixel_buffer 
        global color_name # Ensure these are referenced correctly
         # Reset pixel buffer for each image
        # Stop processing if color has already been detected
        if color_detected:
            return

        # Increment the frame counter
        frame_counter += 1

        # Skip the first 10 frames
        if frame_counter <= 10:
            return

        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Get the dimensions of the image
        height, width, _ = cv_image.shape

        # Find the center pixel coordinates
        center_x, center_y = width // 2, height // 2

        # Collect the BGR values of the center pixel and its neighbors (±1 pixel)
        neighbors = [
            cv_image[center_y + dy, center_x + dx]
            for dy in range(-2, 3)
            for dx in range(-2, 3)
            if 0 <= center_y + dy < height and 0 <= center_x + dx < width
        ]

        # Add the average BGR values of the neighbors to the buffer
        pixel_buffer.append(np.mean(neighbors, axis=0))
        # print("1")
        # If the buffer is full (10 frames after skipping the first 10), compute the average BGR values
        if len(pixel_buffer) == 10:  # Only consider the next 10 frames, as specified
            avg_b, avg_g, avg_r = np.mean(pixel_buffer, axis=0)

            # Determine the color name based on the average RGB values
            color_name = get_color_name(int(avg_r), int(avg_g), int(avg_b))

            # Print the average RGB values and the color name
            rospy.loginfo(f"Average RGB (last 10 frames): R={int(avg_r)}, G={int(avg_g)}, B={int(avg_b)}")
            rospy.loginfo(f"Detected Color: {color_name}")

            # Set the detection flag to True and return the results
            color_detected = True

    # Subscribe to the head camera image topic (reinitialize for each call)
    rospy.Subscriber("/io/internal_camera/head_camera/image_raw", Image, process_image)

    # Wait for detection to complete
    rate = rospy.Rate(10)  # 10 Hz
    while not color_detected and not rospy.is_shutdown():
        # rospy.loginfo("Waiting for color detection...")
        rate.sleep()

    # Cleanup
    try:
        if cameras.is_streaming('head_camera'):
            cameras.stop_streaming('head_camera')
            # rospy.loginfo("Camera stream stopped.")
    except Exception as e:
        rospy.logwarn(f"Failed to stop camera stream: {e}")
    cv2.destroyAllWindows()

    # If color was detected, return the results
    if pixel_buffer and color_detected:
        avg_b, avg_g, avg_r = np.mean(pixel_buffer, axis=0)
        # Clear the buffer explicitly before returning
        pixel_buffer.clear()
        return (int(avg_r), int(avg_g), int(avg_b)), get_color_name(int(avg_r), int(avg_g), int(avg_b))
    else:
        # Clear the buffer explicitly, even if no detection occurred
        pixel_buffer.clear()
        return None, "No Data"

def obj_dect_callback(msg):
    """
    Callback function to handle object detection messages.
    Updates the global variable 'object_detected' based on the message received.
    """
    global object_detected
    object_detected = False  # Reset the flag before processing
    object_detected = msg.data
    if object_detected:
        # rospy.loginfo("Object detected!")
        pub.publish(False)  # Stop the conveyor belt
    else:
        return None  # Exit the loop if no object is detected
        # rospy.loginfo("No object detected.")



def main():
    global limb, object_detected, condition, color_name
    rospy.init_node("record_and_move_end_effector")
    pub.publish(False)
    # Initialize object detection state
    object_detected = False
    head = intera_interface.Head()
    head.set_pan(0)  # Set the head camera to a neutral position
    gripper = intera_interface.Gripper('right')
    gripper.open()
    # Clear any pending images
    time.sleep(1)  # Give some time for the camera to reset

    limb = intera_interface.Limb('right')

    Pickup_position = {
        'right_j0': -0.0310,
        'right_j1': -0.4814,
        'right_j2': -0.1659,
        'right_j3': 1.6506,
        'right_j4': 0.4098,
        'right_j5': 0.4780,
        'right_j6': -0.3799,
        }
    Safe_pickup_position = {
        'right_j0': -0.0179,
        'right_j1': -0.6345,
        'right_j2': -0.1701,
        'right_j3': 1.6808,
        'right_j4': 0.2850,
        'right_j5': 0.5677,
        'right_j6': -0.3402,
    }
    Start_position = {
        'right_j0': -0.7136,
        'right_j1': -0.4938,
        'right_j2': 0.3704,
        'right_j3': 1.4511,
        'right_j4': -0.5668,
        'right_j5': -0.8534,
        'right_j6': -1.3949,
        }

    Head_camera = {
        'right_j0': 0.1718,
        'right_j1': -1.6248,
        'right_j2': -0.3221,
        'right_j3': 1.5965,
        'right_j4': -0.2720,
        'right_j5': 1.8160,
        'right_j6': 4.2934,

    }

    limb.move_to_neutral()

    # Print instructions
    rospy.loginfo("Press Enter to record the Locations In the Following order")
    rospy.loginfo("Cube: Blue, Red, Green, Yellow, Orange, Purple")
    rospy.loginfo("Sphere: Blue, Green, Purple, Orange")
    recorded_angles = record_joint_angles()
    limb.move_to_neutral()    

    keypress = intera_external_devices.getch()
    while keypress != 'c':
        time.sleep(1)
        cv2.destroyAllWindows()
        time.sleep(3)
        condition = ''
        color_name = ''
        
        # Move to start position
        
        move_to_joint_angles(Start_position)

        # STEP 2: Detect shapes from the camera

        shape_detected = detect_shapes_from_camera()
        if shape_detected:
            rospy.loginfo("Shape detected during camera streaming!")
        else:
            rospy.loginfo("No shapes detected during camera streaming.")

        time.sleep(1)

        # Move to Pickup position
        
        # limb.move_to_neutral()
        move_to_joint_angles(Safe_pickup_position)
        move_to_joint_angles(Pickup_position)
        time.sleep(2)
        # Tell Arduino to start the conveyor belt
        
        pub.publish(True)
        print("Conveyor belt started.")

        # Subscribe to Arduino's object detection topic
        rospy.Subscriber("objectDetected", Bool, obj_dect_callback)
        

        # Wait for object detection
        rospy.loginfo("Waiting for object detection...")
        while not object_detected and not rospy.is_shutdown():
            rospy.sleep(0.03)

        rospy.loginfo("Object detected by Arduino!")
        time.sleep(1)
        gripper.close()
        move_to_joint_angles(Safe_pickup_position)
        move_to_joint_angles(Head_camera)

        # STEP 3: Get the color of the object from the camera
        rospy.loginfo("Detecting object color...")
        rgb_values, obj_color = camera_color_detector()
        if rgb_values:
            rospy.loginfo(f"Detected color: {obj_color}, RGB: {rgb_values}")
        else:
            rospy.loginfo("Failed to detect color.")
        time.sleep(1)

        # STEP 4: Move to drop-off location
        rospy.loginfo("Moving to drop-off location...")
        # move_to_start_position()  # Uncomment if you have a specific drop-off location
        print(condition, color_name)
        move_to_angles(recorded_angles, condition, color_name)
        gripper.open()
        limb.move_to_neutral()

if __name__ == "__main__":
    main()
