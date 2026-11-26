There is a Raspberry PI 5 at IP address 192.168.1.80 running Raspberry Pi OS Trixie. The Pi has an Adafruit 16-channel servo motor hat attached. I have enabled I2C communications and have successfully tested communications. I also have a USB webcam connected to the Pi. The webcam has been successfully tested using the "cheese" program. The webcam is attached to a pan-and-tilt structure driven by two servo motors. The pan servo is connected to channel 0 and the tilt servo is attached to channel 1. 

I would like to create a program that reads the video feed from the camera, detects the faces in the scene, determines which face bounding box is the largest and then moves the pan and tilt servos to move the largest face towards the centre of the frame. 

I would like to use OpenCV for the image manipulation, the Haar Cascade for face detection. 

Please help me: 
1) install the python library needed to talk to the servo motors, 
2) install OpenCV,
3) install the Haar Cascade, 
4) write the code to detect faces and move the servo motors to center the largest face. 

