# 1. Introduction

This project focuses on developing a ROS2-based autonomous docking system using ArUco markers as visual fiducials. The objective was to design, build, and validate a workflow that enables a mobile robot to detect an ArUco marker, estimate its pose, and navigate toward it for docking. This task evaluated technical understanding of ROS2 (Humble), robot perception, TF transformations, URDF description, and autonomous navigation pipeline.

---

# 2. System Architecture

The solution was designed inside a ROS2 workspace (`ros2_ws`) and organized into modular packages.

## 2.1 robot_description
- Contains robot URDF/Xacro file  
- RViz visualization configuration  
- TF frames for `base_link` and `camera_link`  
- Launch files for visualization  

## 2.2 aruco_detector
- Subscribes to `camera/image_raw` & `camera_info`  
- Uses OpenCV ArUco dictionary for marker detection  

---

# 3. Key Functional Components

## 3.1 Perception (ArUco Detection)
- ArUco marker detected using `cv2.aruco.detectMarkers()`  
- Camera intrinsics loaded from a YAML calibration file  
- Pose estimation via `cv2.aruco.estimatePoseSingleMarkers()`  
- TF broadcaster publishes `marker_frame`  

## 3.2 TF Transformations

**robot_description** publishes static transforms:
- `base_link → camera_link`

**aruco_detector** publishes dynamic transforms:
- `camera_link → aruco_marker`

RViz is used to validate these transforms, with the fixed frame set to `base_link`.

---

# 4. Testing and Validation

## 4.1 RViz Visualization
- Marker pose confirmed through TF tree inspection  
- Robot model displayed correctly (URDF validated)  

---

# 5. Insights Gained

## 5.1 Importance of TF Alignment
Incorrect TF frames result in:
- No fixed frame in RViz  
- Incorrect pose estimation  
- Controller oscillation  

Correcting the TF tree was essential for stable performance.

## 5.2 ROS2 Package Structure Matters
- Proper `CMakeLists.txt` and `package.xml` prevent build errors  
- Keeping `robot_description`, `aruco_detector`, and control logic modular simplifies debugging  

---

# 6. Conclusion

The project successfully implemented a working ROS2-based autonomous docking system using ArUco markers. It involved understanding robot description files, TF frames, perception pipelines, and control strategies. The final implementation is modular, extensible, and well-suited as a foundation for advanced tasks such as visual servoing, navigation stack integration, or multi-marker docking systems.

