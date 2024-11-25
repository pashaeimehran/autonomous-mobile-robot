# autonomous-mobile-robot
Autonomous Navigation Mobile Robot using ROS Navigation Stack

---

# Autonomous Navigation Mobile Robot using ROS Navigation Stack

## 📜 Project Overview  
This project involves the design and implementation of an autonomous mobile robot capable of navigating a predefined environment using the ROS (Robot Operating System) Navigation Stack. The system integrates advanced sensors, efficient path planning algorithms, and real-time data processing to achieve precise and reliable navigation.

The project was part of my bachelor's thesis in Electrical and Electronics Engineering and showcases my expertise in embedded systems programming, robotics, and algorithm development.

---

## 🚀 Features  
- Autonomous Navigation Path planning and obstacle avoidance using ROS Navigation Stack.  
- Sensor Integration Utilizes lidar, encoders for real-time data acquisition.  
- Localization and Mapping Implements SLAM (Simultaneous Localization and Mapping) for dynamic environments.  
- Modular Code Organized ROS nodes for control, navigation, and sensor handling.  
- Expandable Framework Easily adaptable to new environments and sensor configurations.

---

## 🛠️ System Components  

### Hardware  
- Microcontroller Raspberry Pi 4B  
- Sensors  
  - Lidar X2 for obstacle detection.     
- Motor Drivers Dual DC motors.  
- Power Supply Lipo Battery.  

### Software  
- ROS Framework ROS Noetic for robot operating system functionalities.  
- Programming Languages Python.  
- Path Planning Algorithm DijkstraA, integrated with ROS Navigation Stack.    

---

## 📂 Repository Structure  
```
├── Motor_Driver       # Source codes
├── Remote_Control     # Source codes
├── SLAM               # Configuration files for robot and sensors
├── yaml               # Configuration files for robot and sensors
├── media              # Images, videos, and documentation
└── README.md           # Project description
```

---

## 📜 Prerequisites  

### Hardware  
- Properly assembled mobile robot with lidar, and motors.  

### Software  
- ROS MelodicNoetic installed on Linux (Ubuntu).  
- Dependencies  
  - `move_base`  
  - `amcl`  
  - `gmapping`  
  - `rviz`  
- Python libraries `numpy`, `rospy`.  


## 📈 Future Improvements  
- Integrating advanced sensors like 3D lidar or depth cameras.  
- Improving localization with RTK GPS for outdoor environments.  
- Adding a web-based interface for manual control and monitoring.

---

## 🏆 Acknowledgments  
Special thanks to Prof. Dr. Musa Alcı for supervision and guidance throughout the project.

---


## 📞 Contact  
Feel free to reach out if you have questions or suggestions!  
📧 Email mehran.pashaei.mp@gmail.com  
🔗 [LinkedIn](httpswww.linkedin.cominmehran-pashaei-b582ba208)  
