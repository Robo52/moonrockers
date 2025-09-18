#!/usr/bin/env python3
"""
Vision System for rover
Handles ArUco marker detection and obstacle detection using RealSense D435i
John Miller
"""

import cv2
import numpy as np
import time
import threading
from typing import List, Tuple, Optional, Dict, Callable
from dataclasses import dataclass
import logging
from enum import Enum

# RealSense camera support
        try:
            import pyrealsense2 as rs
            REALSENSE_AVAILABLE = True
        except ImportError:
            REALSENSE_AVAILABLE = False
            logger.error("pyrealsense2 not available - install with: pip install pyrealsense2")
            logger.error("RealSense camera will not function without this library")

logger = logging.getLogger(__name__)

@dataclass
class ArUcoMarker:
    id: int
    corners: np.ndarray
    center: Tuple[float, float]
    distance: float
    angle: float  # Angle relative to camera center
    pose_vector: Optional[np.ndarray] = None  # Translation vector
    pose_rotation: Optional[np.ndarray] = None  # Rotation vector

@dataclass
class ObstacleInfo:
    distance: float
    angle: float  # Angle from camera center
    size: float   # Angular size in degrees
    center_pixel: Tuple[int, int]

class VisionMode(Enum):
    ARUCO_DETECTION = "aruco"
    OBSTACLE_DETECTION = "obstacles"
    COMBINED = "combined"

class VisionSystem:
    """
    Complete vision system for rover navigation and localization
    Combines ArUco detection and obstacle detection
    """
    
    def __init__(self, config: dict):
        """
        Initialize vision system
        
        Args:
            config: Configuration dictionary containing vision settings
        """
        self.config = config['VISION_CONFIG']
        
        # Camera parms
        self.camera_matrix = self.config.get('camera_matrix')
        self.distortion_coeffs = self.config.get('distortion_coeffs')
        self.aruco_size = self.config['aruco_size']  # Size in meters
        
        # ArUco detection setup
        aruco_dict_name = self.config['aruco_dict']
        self.aruco_dict = getattr(cv2.aruco, aruco_dict_name)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera and pipeline
        self.pipeline = None
        self.camera_running = False
        self.current_frame = None
        self.current_depth = None
        self.frame_lock = threading.Lock()
        
        # Detection results
        self.detected_markers = []
        self.detected_obstacles = []
        self.last_detection_time = 0
        
        # Callbacks for detected objects
        self.on_marker_detected = None
        self.on_obstacle_detected = None
        self.on_frame_processed = None
        
        # Processing parameters
        self.obstacle_min_distance = 0.1  # meters
        self.obstacle_max_distance = 5.0  # meters
        self.obstacle_detection_threshold = 100  # depth difference threshold
        
        logger.info("Vision system initialized")
    
    def initialize_camera(self):
        """Initialize RealSense camera"""
        if not REALSENSE_AVAILABLE:
            logger.error("RealSense library not available - cannot initialize camera")
            logger.error("Install with: pip install pyrealsense2")
            return False
        
        try:
            # Config RealSense pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Config streams
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get camera intrinsics
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            self.camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            
            self.distortion_coeffs = np.array(intrinsics.coeffs)
            
            logger.info(f"RealSense camera initialized successfully")
            logger.info(f"Resolution: 640x480 @ 30fps")
            logger.info(f"Intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
            
            # Allow camera to stabilize
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            self.camera_running = True
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize RealSense camera: {e}")
            logger.error("Ensure RealSense D435i is connected via USB 3.0")
            logger.error("Check that no other application is using the camera")
            return False

    
    def capture_frame(self) -> bool:
        """Capture a frame from the RealSense camera"""
        if not self.camera_running or not self.pipeline:
            logger.error("Camera not running or not initialized")
            return False
        
        try:
            # RealSense capture with timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame:
                logger.warning("No color frame received")
                return False
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()) if depth_frame else None
            
            if depth_image is None:
                logger.warning("No depth frame received")
            
            with self.frame_lock:
                self.current_frame = color_image.copy()
                self.current_depth = depth_image.copy() if depth_image is not None else None
            
            return True
            
        except Exception as e:
            logger.error(f"Frame capture failed: {e}")
            return False
    
    def detect_aruco_markers(self, frame: np.ndarray) -> List[ArUcoMarker]:
        """
        Detect ArUco markers in the given frame
        
        Args:
            frame: Input image frame
            
        Returns:
            List of detected ArUco markers
        """
        if frame is None:
            return []
        
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            
            markers = []
            
            if ids is not None and len(ids) > 0:
                # Estimate poses
                rvecs = None
                tvecs = None
                
                if self.camera_matrix is not None and self.distortion_coeffs is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.aruco_size, self.camera_matrix, self.distortion_coeffs
                    )
                
                # Process each detected marker
                for i, marker_id in enumerate(ids.flatten()):
                    corner_set = corners[i][0]
                    
                    # Calculate center point
                    center_x = np.mean(corner_set[:, 0])
                    center_y = np.mean(corner_set[:, 1])
                    center = (center_x, center_y)
                    
                    # Calculate distance and angle
                    distance = 0.0
                    angle = 0.0
                    pose_vec = None
                    pose_rot = None
                    
                    if tvecs is not None and rvecs is not None:
                        # Distance from pose estimation
                        pose_vec = tvecs[i][0]
                        pose_rot = rvecs[i][0]
                        distance = np.linalg.norm(pose_vec)
                        
                        # Angle relative to camera center
                        frame_center_x = frame.shape[1] / 2
                        pixel_angle_per_pixel = 60.0 / frame.shape[1]  # Rough estimate
                        angle = (center_x - frame_center_x) * pixel_angle_per_pixel
                    
                    marker = ArUcoMarker(
                        id=int(marker_id),
                        corners=corner_set,
                        center=center,
                        distance=distance,
                        angle=angle,
                        pose_vector=pose_vec,
                        pose_rotation=pose_rot
                    )
                    
                    markers.append(marker)
                    
                    logger.debug(f"Detected ArUco {marker_id}: distance={distance:.2f}m, angle={angle:.1f}°")
            
            return markers
            
        except Exception as e:
            logger.error(f"ArUco detection failed: {e}")
            return []
    
    def detect_obstacles(self, color_frame: np.ndarray, depth_frame: Optional[np.ndarray]) -> List[ObstacleInfo]:
        """
        Detect obstacles using depth information
        
        Args:
            color_frame: Color image frame
            depth_frame: Depth image frame (None if not available)
            
        Returns:
            List of detected obstacles
        """
        if depth_frame is None:
            return []
        
        obstacles = []
        
        try:
            # Convert depth to float and filter
            depth_float = depth_frame.astype(np.float32) / 1000.0  # Convert mm to meters
            
            # Create mask for valid depth range
            valid_mask = (depth_float > self.obstacle_min_distance) & \
                        (depth_float < self.obstacle_max_distance)
            
            # Find obstacles by looking for depth discontinuities
            # Using horizontal and vertical gradients
            grad_x = cv2.Sobel(depth_float, cv2.CV_32F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(depth_float, cv2.CV_32F, 0, 1, ksize=3)
            
            # Combine gradients
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
            
            # Threshold to find edges (obstacles)
            obstacle_mask = (gradient_magnitude > 0.1) & valid_mask
            
            # Find contours of obstacle regions
            obstacle_mask_uint8 = obstacle_mask.astype(np.uint8) * 255
            contours, _ = cv2.findContours(obstacle_mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            frame_center_x = color_frame.shape[1] / 2
            pixel_angle_per_pixel = 60.0 / color_frame.shape[1]  # Rough fov might need to change
            
            for contour in contours:
                # Filter small contours
                if cv2.contourArea(contour) < 100:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Get distance at center point
                if 0 <= center_y < depth_frame.shape[0] and 0 <= center_x < depth_frame.shape[1]:
                    distance = depth_float[center_y, center_x]
                    
                    if distance > 0:  # Valid depth measurement
                        # Calculate angle from camera center
                        angle = (center_x - frame_center_x) * pixel_angle_per_pixel
                        
                        # Calculate angular size
                        angular_size = w * pixel_angle_per_pixel
                        
                        obstacle = ObstacleInfo(
                            distance=distance,
                            angle=angle,
                            size=angular_size,
                            center_pixel=(center_x, center_y)
                        )
                        
                        obstacles.append(obstacle)
                        
                        logger.debug(f"Obstacle detected: distance={distance:.2f}m, angle={angle:.1f}°")
            
        except Exception as e:
            logger.error(f"Obstacle detection failed: {e}")
        
        return obstacles
    
    def process_frame(self, mode: VisionMode = VisionMode.COMBINED) -> bool:
        """
        Process current frame for markers and/or obstacles
        
        Args:
            mode: Processing mode (ArUco, obstacles, or combined)
            
        Returns:
            True if processing was successful
        """
        with self.frame_lock:
            if self.current_frame is None:
                return False
            
            frame = self.current_frame.copy()
            depth = self.current_depth.copy() if self.current_depth is not None else None
        
        try:
            # Detect ArUco markers
            if mode in [VisionMode.ARUCO_DETECTION, VisionMode.COMBINED]:
                self.detected_markers = self.detect_aruco_markers(frame)
                
                # Trigger callback
                if self.on_marker_detected and self.detected_markers:
                    self.on_marker_detected(self.detected_markers)
            
            # Detect obstacles
            if mode in [VisionMode.OBSTACLE_DETECTION, VisionMode.COMBINED]:
                self.detected_obstacles = self.detect_obstacles(frame, depth)
                
                # Trigger callback
                if self.on_obstacle_detected and self.detected_obstacles:
                    self.on_obstacle_detected(self.detected_obstacles)
            
            self.last_detection_time = time.time()
            
            # Frame processed callback
            if self.on_frame_processed:
                self.on_frame_processed(frame, self.detected_markers, self.detected_obstacles)
            
            return True
            
        except Exception as e:
            logger.error(f"Frame processing failed: {e}")
            return False
    
    def draw_detections(self, frame: np.ndarray) -> np.ndarray:
        """
        Draw detected markers and obstacles on frame
        
        Args:
            frame: Input frame
            
        Returns:
            Frame with detections drawn
        """
        output_frame = frame.copy()
        
        try:
            # Draw ArUco markers
            for marker in self.detected_markers:
                # Draw marker outline
                cv2.aruco.drawDetectedMarkers(output_frame, [marker.corners.reshape(1, -1, 2)])
                
                # Draw marker info
                center = (int(marker.center[0]), int(marker.center[1]))
                cv2.circle(output_frame, center, 5, (0, 255, 0), -1)
                
                # Draw text info
                text = f"ID:{marker.id} D:{marker.distance:.2f}m A:{marker.angle:.1f}°"
                cv2.putText(output_frame, text, (center[0] + 10, center[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw obstacles
            for obstacle in self.detected_obstacles:
                center = obstacle.center_pixel
                radius = max(10, int(obstacle.size * 2))
                
                # Draw obstacle circle
                cv2.circle(output_frame, center, radius, (0, 0, 255), 2)
                
                # Draw obstacle info
                text = f"Obs D:{obstacle.distance:.2f}m A:{obstacle.angle:.1f}°"
                cv2.putText(output_frame, text, (center[0] + radius + 5, center[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
            # Draw frame info
            info_text = f"Markers: {len(self.detected_markers)}, Obstacles: {len(self.detected_obstacles)}"
            cv2.putText(output_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        except Exception as e:
            logger.error(f"Drawing detections failed: {e}")
        
        return output_frame
    
    def run_continuous_detection(self, mode: VisionMode = VisionMode.COMBINED, 
                                fps: int = 10, display: bool = False):
        """
        Run continuous detection in a separate thread
        
        Args:
            mode: Detection mode
            fps: Target frames per second
            display: Show detection results in window
        """
        def detection_loop():
            frame_time = 1.0 / fps
            
            while self.camera_running:
                start_time = time.time()
                
                # Capture and process frame
                if self.capture_frame():
                    self.process_frame(mode)
                    
                    # Display if requested
                    if display and self.current_frame is not None:
                        display_frame = self.draw_detections(self.current_frame)
                        cv2.imshow('Rover Vision', display_frame)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                
                # Maintain frame rate
                elapsed = time.time() - start_time
                if elapsed < frame_time:
                    time.sleep(frame_time - elapsed)
        
        # Start detection thread
        detection_thread = threading.Thread(target=detection_loop, daemon=True)
        detection_thread.start()
        
        logger.info(f"Continuous detection started at {fps} FPS")
        return detection_thread
    
    def get_latest_detections(self) -> Tuple[List[ArUcoMarker], List[ObstacleInfo]]:
        """Get the latest detection results"""
        return self.detected_markers.copy(), self.detected_obstacles.copy()
    
    def find_marker_by_id(self, marker_id: int) -> Optional[ArUcoMarker]:
        """Find a specific marker by ID"""
        for marker in self.detected_markers:
            if marker.id == marker_id:
                return marker
        return None
    
    def get_closest_obstacle(self) -> Optional[ObstacleInfo]:
        """Get the closest detected obstacle"""
        if not self.detected_obstacles:
            return None
        
        return min(self.detected_obstacles, key=lambda obs: obs.distance)
    
    def shutdown(self):
        """Shutdown vision system"""
        logger.info("Shutting down vision system")
        
        self.camera_running = False
        
        if self.pipeline:
            self.pipeline.stop()
        
        cv2.destroyAllWindows()


# Test
if __name__ == "__main__":
    from config import VISION_CONFIG
    
    config = {'VISION_CONFIG': VISION_CONFIG}
    
    def marker_callback(markers):
        print(f"Detected {len(markers)} markers:")
        for marker in markers:
            print(f"  ID {marker.id}: distance={marker.distance:.2f}m, angle={marker.angle:.1f}°")
    
    def obstacle_callback(obstacles):
        if obstacles:
            closest = min(obstacles, key=lambda obs: obs.distance)
            print(f"Closest obstacle: {closest.distance:.2f}m at {closest.angle:.1f}°")
    
    try:
        # Init
        vision = VisionSystem(config)
        
        if not vision.initialize_camera():
            print("Failed to initialize camera")
            exit(1)
        
        # Set callbacks
        vision.on_marker_detected = marker_callback
        vision.on_obstacle_detected = obstacle_callback
        
        print("Vision system test starting...")
        print("Press 'q' in the display window to quit")
        
        detection_thread = vision.run_continuous_detection(
            mode=VisionMode.COMBINED,
            fps=10,
            display=True
        )
        
        # Keep main thread alive
        try:
            while vision.camera_running:
                time.sleep(0.1)
                
                # Show latest detections every 2 seconds
                if int(time.time()) % 2 == 0:
                    markers, obstacles = vision.get_latest_detections()
                    if markers or obstacles:
                        print(f"Current: {len(markers)} markers, {len(obstacles)} obstacles")
                        
        except KeyboardInterrupt:
            print("\nStopping vision system...")
        
        vision.shutdown()
        print("Vision system test completed!")
        
    except Exception as e:
        print(f"Test failed: {e}")
        if 'vision' in locals():
            vision.shutdown()
