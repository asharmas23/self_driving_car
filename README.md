
# DeepC CAR: A Self-Driving Car Project

## Project Overview
DeepC CAR is an innovative project that brings the dream of self-driving cars into reality. Developed using Convolutional Neural Networks (CNNs), this project aims at controlling a car's throttle and steering angle based on real-time camera images. The implementation spans both a hardware prototype and a virtual simulation within the GTA V game environment, showcasing the versatility and potential of the developed model.

## Features
- **Real-Time Control:** Utilizes a CNN to predict throttle values and steering angles from camera feeds.
- **Obstacle Detection:** Incorporates ultrasonic sensors for real-time obstacle detection and avoidance.
- **Dual Implementation:** Demonstrated in the real world with a hardware prototype and virtually within GTA V game.
- **High Accuracy:** Achieves ~91% accuracy on the validation set, ensuring reliable performance.

## Components
- **Arduino Uno:** Serves as the project's microcontroller.
- **Ultrasonic Sensors (SR-04):** For distance measurement and obstacle detection.
- **USB Webcam:** Captures live footage for real-time processing by the CNN.

## Demo Videos

- **Real-World Implementation**
[![Real-World Implementation](https://drive.google.com/uc?export=view&id=1pDWPB8knDXa2WCWWj3djhAS_atI9jSaI)](https://drive.google.com/file/d/1pDWPB8knDXa2WCWWj3djhAS_atI9jSaI/view?usp=sharing "Click to View: Real-World Implementation")

- **GTA V Simulation**
[![GTA V Simulation](https://drive.google.com/uc?export=view&id=1Rc9Xi1lZeiP6bGrhhp7zJr4KzAZYxp8h)](https://drive.google.com/file/d/1Rc9Xi1lZeiP6bGrhhp7zJr4KzAZYxp8h/view?usp=drive_link "Click to View: GTA V Simulation")

- **Additional Demonstrations**
[![Additional Demonstrations](https://drive.google.com/uc?export=view&id=1PobrqCokJKmt-AabfhVA2xrdnMUoq9s7)](https://drive.google.com/file/d/1PobrqCokJKmt-AabfhVA2xrdnMUoq9s7/view?usp=drive_link "Click to View: Additional Demonstrations")


## Use Cases
1. **Hardware Prototype:** A physical self-driving car model that navigates real-world environments.
2. **GTA V Control:** Virtual demonstration of the CNN's applicability in controlling a vehicle within the GTA V game, using game frames as input.

## Potential Impact
The DeepC CAR project aims to revolutionize mobility, offering safer transportation options, reducing commute times, and making autonomous driving accessible to disabled and elderly individuals. It highlights the broader applicability of AI in enhancing vehicle safety, efficiency, and independence.

## Future Directions
Continual improvements on model accuracy, adoption of LIDAR for enhanced obstacle detection in adverse weather, and progression towards higher levels of driving autonomy (Level 4).

## Conclusion
DeepC CAR exemplifies how deep learning can pave the way for autonomous vehicles, potentially reducing road accidents and providing mobility solutions for millions.

## References
- [Arduino Uno](https://www.arduino.cc/en/Main/ArduinoBoardUno)
- [End to End Learning for Self-Driving Cars](https://arxiv.org/abs/1604.07316)
- [Udacity Self-Driving Car Simulator](https://github.com/udacity/self-driving-car-sim)
