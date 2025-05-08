# Object Sorting Conveyor System with Robotic Arm

This project involves a conveyor belt system equipped with sensors and a robotic arm to detect, identify, and sort objects based on specific criteria. It integrates Arduino-based control with Python, OpenCV, and ROS for full automation.

---

## 🔧 Hardware Required

| Component | Description | Datasheet / Link |
|----------|-------------|------------------|
| **Arduino Uno R3** | Microcontroller for sensor and relay control | [Datasheet](https://docs.arduino.cc/resources/datasheets/A000066-datasheet.pdf) |
| **KEYESIR IR Sensor KY-032** | Used to detect objects on the conveyor | [Datasheet](http://irsensor.wizecode.com/sensor.pdf) |
| **24V Arduino-Compatible Relay Module** | Controls conveyor motor power | [Datasheet](https://handsontec.com/dataspecs/module/Relay%20Module/1Ch-relay%20Opto.pdf) |
| **24V Power Supply** | Powers the conveyor motor |
| **Dobot Conveyor Belt** | The physical conveyor system |
| **Rethink Robotics Sawyer 7-DOF Robotic Arm** | Performs the sorting actions | [Sawyer Support Page](https://support.rethinkrobotics.com/support/solutions/80000457340) |
| **Laptop with Ubuntu 20.04** | Hosts Python, ROS, and vision processing scripts |

---

## 💻 Software Requirements

### Main Script (Python)
- Python 3
- OpenCV (`cv2`)
- `time` module
- `math` module
- `rospy`
- `rosserial_python`

### Arduino Software
- Arduino IDE
- `rosserial_arduino` library

### Supplementary ROS Packages
Install via terminal:
sudo apt install ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rosserial-python

### Arduino Wiring Diagram 
![Arduino Wiring Diagram](https://github.com/user-attachments/assets/ebd2cf71-676a-4389-b424-fa3caf27992b)
