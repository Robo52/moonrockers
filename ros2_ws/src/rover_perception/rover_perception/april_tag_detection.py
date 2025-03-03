#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import apriltag
import math

class AprilTagPoseEstimator(Node):
    def __init__(self):
        super().__init__('apriltag_pose_estimator')
        self.get_logger().info("AprilTag Pose Estimator Node Starting...")
        self.bridge = CvBridge()

        # Initialize camera intrinsics with default D435i values;
        # These will be updated by the CameraInfo callback.
        self.fx = 615.0  # placeholder; update from /cam_1/color/camera_info
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0

        # Set the physical tag size (meters) of the inner (data/black) square.
        # Adjust this to measured tag size
        self.tag_size = 0.15

        # Subscribe to CameraInfo to update intrinsics
        self.create_subscription(
            CameraInfo,
            '/cam_1/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to the rectified image topic
        self.create_subscription(
            Image,
            '/cam_1/color/image_rect',  # Ensure this is the topic publishing rectified images may differ with Realsense Node
            self.image_callback,
            10
        )

        # Publisher for the detected tag's pose
        self.pose_pub = self.create_publisher(PoseStamped, '/apriltag_pose', 10)

        # Configure the AprilTag detector to use tagStandard41h12 family
        options = apriltag.DetectorOptions(families="tagStandard41h12", refine_edges=True)
        self.detector = apriltag.Detector(options)

    def camera_info_callback(self, msg: CameraInfo):
        # Update intrinsics dynamically from the camera info message.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().debug(f"Camera intrinsics updated: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def image_callback(self, msg: Image):
        try:
            # Convert the rectified image to grayscale
            gray_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Detect AprilTags in the image
        detections = self.detector.detect(gray_img)
        if len(detections) == 0:
            self.get_logger().debug("No AprilTags detected in this frame.")
            return

        # For detected tag, compute pose relative to the camera.
        for detection in detections:
            tag_id = detection.tag_id
            self.get_logger().info(f"Detected tag id: {tag_id}")

            # Image points: use the four detected corners.
            # Expected order: [top-left, top-right, bottom-right, bottom-left]
            image_points = np.array(detection.corners, dtype=np.float32)

            # Define the 3D coordinates of the tag's corners in the tag's coordinate frame.
            # Assume the tag lies in the XY plane with Z=0 and the tag is centered at the origin.
            s = self.tag_size
            object_points = np.array([
                [-s/2,  s/2, 0],
                [ s/2,  s/2, 0],
                [ s/2, -s/2, 0],
                [-s/2, -s/2, 0]
            ], dtype=np.float32)

            # Build the camera matrix using the intrinsics
            camera_matrix = np.array([
                [self.fx,    0, self.cx],
                [   0,  self.fy, self.cy],
                [   0,       0,      1]
            ], dtype=np.float32)

            # Assume no distortion (the image is rectified)
            dist_coeffs = np.zeros((4, 1), dtype=np.float32)

            # Solve for the rotation and translation vectors using solvePnP.
            # Use cv2.SOLVEPNP_IPPE_SQUARE, good for square fiducials.
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                self.get_logger().warn(f"solvePnP failed for tag id {tag_id}")
                continue

            # Convert the rotation vector to a 3x3 rotation matrix.
            R, _ = cv2.Rodrigues(rvec)

            # Convert the rotation matrix to a quaternion (x, y, z, w)
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

            # Create and populate the PoseStamped message.
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            # We can override the frame_id:
            # pose_msg.header.frame_id = "camera_link"
            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pose_pub.publish(pose_msg)
            self.get_logger().info(
                f"Published pose for tag id {tag_id}: position {tvec.ravel()}, "
                f"orientation (qx, qy, qz, qw): ({qx}, {qy}, {qz}, {qw})"
            )

    def rotation_matrix_to_quaternion(self, R: np.ndarray):
        """
        Convert a 3x3 rotation matrix R to a quaternion (qx, qy, qz, qw)
        following the ROS (x, y, z, w) convention.
        """
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
