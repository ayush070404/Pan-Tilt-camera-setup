# **Pan-Tilt-camera-setup**


Developing 3D printable design and implementing pan/ tilt setup for object tracking camera system from scratch


Creating a 3D printable design and implementing a pan/tilt setup for an object tracking camera system from scratch involves several steps. The process includes 3D modeling, 3D printing, assembly, and programming the tracking system. Here’s a comprehensive guide to achieving this:

### 1. 3D Modeling

You will need to design the pan/tilt mechanism using 3D modeling software. Popular choices include Fusion 360, Tinkercad, or Blender. The design will typically consist of two main parts:
- **Base**: For the pan (horizontal movement)
- **Tilt**: For the tilt (vertical movement)

#### Base Design:
1. **Create a circular base** with a central hole to mount a servo motor.
2. **Design a platform** that attaches to the servo horn, allowing the platform to rotate horizontally.

#### Tilt Design:
1. **Design a vertical bracket** that attaches to the base platform.
2. **Create a mounting bracket** for the camera that attaches to another servo for vertical movement.

### 2. 3D Printing

Once the design is complete, export the parts as STL files and use a 3D printer to print them. Ensure your design accounts for the dimensions and mounting points for the servo motors and the camera.

### 3. Assembly

You will need:
- 2x SG90 or MG90S servo motors (one for pan, one for tilt)
- Screws and nuts to attach the servos to the printed parts
- A small camera module (e.g., Raspberry Pi Camera or a USB webcam)
- A microcontroller (e.g., Arduino, Raspberry Pi)

#### Steps:
1. **Attach the first servo motor** to the base platform. This servo controls the horizontal movement (pan).
2. **Mount the vertical bracket** on the rotating part of the pan servo.
3. **Attach the second servo motor** to the vertical bracket. This servo controls the vertical movement (tilt).
4. **Mount the camera** to the tilting bracket attached to the tilt servo.

### 4. Wiring

Connect the servos to the microcontroller. For example, with an Arduino:
- Connect the **servo signal wires** to PWM-capable digital pins (e.g., pins 9 and 10).
- Connect the **servo power wires** to the 5V and GND pins of the Arduino.

### 5. Programming

Write the code to control the servos and implement object tracking. You can use a combination of OpenCV for object detection and a microcontroller library to control the servos.

#### Example with OpenCV and Arduino:

##### OpenCV (Python):
1. **Install OpenCV**: `pip install opencv-python`
2. **Object detection**: Use a simple color-based tracking or a pre-trained model for object detection.



1. **OpenCV Script**:
   - Captures frames from the webcam.
   - Converts the frames to grayscale and applies Gaussian blur for noise reduction.
   - Uses thresholding to detect objects (can be replaced with more sophisticated detection).
   - Draws bounding boxes around detected objects and calculates the center.
   - Sends the center coordinates to the Arduino via serial communication.

2. **Arduino Script**:
   - Reads the coordinates from the serial port.
   - Maps the coordinates to servo angles.
   - Controls the servos to point the camera towards the detected object.

### Summary
This project requires designing, printing, assembling the pan/tilt mechanism, and writing software to control it. By combining OpenCV for object detection and an Arduino for servo control, you can create a functional object tracking camera system. Make sure to test each component individually before integrating them into the final system.
