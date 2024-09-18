# FlawFactory

![FlawFactory Logo](/media/flawfactory.png)

[![WRO - Future Engineers](https://img.shields.io/badge/WRO-Future_Engineers-2e52af)](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)
[![YouTube - Opening Race](https://img.shields.io/badge/YouTube-▶️%20Opening_Race-df3e3e?logo=youtube)](https://www.youtube.com/watch?v=waIHJP2l4eQ)
[![YouTube - Obstacle Race](https://img.shields.io/badge/YouTube-▶️%20Obstacle_Race-df3e3e?logo=youtube)](https://www.youtube.com/watch?v=hp4pJUoMnfw)

**This is the github repository for team FlawFactory for WRO 2024 Brescia. You'll find our documentation in this readme**

## Contents

- [Mobility and Hardware Design](#Mobility-Management)
- [Power and Sense Management](#Power-and-Sense-Management)
  - [Power Management](#Power-Management)
  - [Sense Management](#Sense-Management)
  - [Wiring Diagram](#Wiring-Diagram)
  - [Bill of Materials](#Bill-of-Materials)
- [Obstacle Management](#Obstacle-Management)
- [Photos](#Photos)
- [Videos](#Videos)
- [Enabling Reproducibility](#Enabling-Reproducibility)

<!-- Mobility management discussion should cover how the vehicle movements are managed. What motors are selected, how they are selected and implemented.
A brief discussion regarding the vehicle chassis design /selection can be provided as well as the mounting of all components to the vehicle chassis/structure. The discussion may include engineering principles such as speed, torque, power etc. usage. Building or assembly instructions can be provided together with 3D CAD files to 3D print parts. -->

## Mobility Management

The robot is built on a Lego chassis, which is supplemented where necessary with custom-designed 3D-printed components. These parts securely attach the motors and electronics to the chassis. The Lego structure incorporates dual Ackermann steering mechanism for both axles, allowing the robot to navigate tight turns with ease. To ensure even distribution of rotational power, we integrated a differential gear into each axle, powering all four wheels.

We connected the motors to the chassis using 3D-printed couplings that link the motor shaft directly to a Lego axle, allowing a single DC motor to drive the entire robot. The motor is positioned as low as possible to achieve a stable structure with a low center of gravity. We selected a 12V, 251 RPM motor with a 43.8:1 gearbox, which, despite its modest speed, is ideal for the robot due to its high torque of 18 kg\*cm at a stall current of 7A. This powerful motor allows for smooth driving and acceleration. The steering mechanism utilizes an RC servo motor with a stall torque of 15 kg\*cm, connected to the steering axle via a 3D-printed coupling and Lego gears. Since the DC motor obstructed the direct connection between the front and rear axles for simultaneous steering, we extended the linkage above the motor, placing the servo motor on top of the robot.

For the remaining electronics, we designed and printed additional parts. The camera mount is designed to allow the camera to be easily inserted from the top. The other electronics are mounted on a two-story plate, which is attached to the chassis using the classic Lego method of pushing axles through the 3D-printed parts and beams. We used standard M3 or M2.5 hardware to assemble the 3D-printed components and secure the electronics. The parts were designed to allow nuts to be pressed into place without support structures during printing, enabling a strong and reliable connection. Depending on accessibility, we used either square or hex nuts for the assembly.

<!-- Power and Sense management discussion should cover the power source for the vehicle as well as the sensors required to provide the vehicle with information to negotiate the different challenges. The discussion can include the reasons for selecting various sensors and how they are being used on the vehicle together with power consumption. The discussion could include a wiring diagram with BOM for the vehicle that includes all aspects of professional wiring diagrams. -->

## Power and Sense Management

### Power Management

We utilize a standard 4S LiPo battery as our power source. This versatile, off-the-shelf solution enables us to power both our drive and computer vision subsystems using a DC-DC buck converter. The 14.8V from the battery is directly fed into the H-Bridge motor driver, specifically an L298N variant. The buck converter provides a stable 5V supply to the main onboard computer, a Raspberry Pi 4B, which runs our computer vision algorithms. The 5V output is also routed to an Arduino Nano, connected via USB serial to the Raspberry Pi, to separate vision processing from motor control. The Arduino Nano handles tasks such as acceleration and PWM modulation.

Power consumption was a significant consideration, as the Raspberry Pi can draw up to 1.2 amps, and the servo motor can pull up to 4A in a stall situation. The DC-DC converter was selected to meet these requirements. The drive motor, powered separately, can draw up to 7A. By separating the power sources, only the H-Bridge needed to be rated for these high currents.

Due to cost considerations, we selected an H-Bridge with a current rating below the stall current, necessitating gradual acceleration to avoid damage—a lesson learned through experience. The H-Bridge allows the Arduino Nano to control the motor with three pins: two for selecting the direction of rotation and one for moderating speed using PWM (pulse width modulation). This setup ensures that the 12V system can be safely controlled using the 5V logic from the Raspberry Pi and Arduino.

### Sense Management

For the regional finals, we experimented with various sensors, including ultrasonic sensors, gyroscopes, and cameras. Through our trials, we discovered that a camera alone suffices, provided it has a sufficiently wide field of view. Consequently, we selected a Pi HQ camera equipped with a screw-on lens offering a 120° field of view. This camera allows the robot to accurately perceive its surroundings.

The Pi HQ camera connects directly to the Raspberry Pi, drawing power from the Pi itself, which is supplied via the 5V pins on the GPIO connector. Based on our experiences at the regional finals, we decided to eliminate all ultrasonic sensors and the gyroscope. Instead of relying on hardware sensors, we opted to enhance video processing to emulate distance sensing. To facilitate this and improve wall detection robustness, we mounted the camera precisely 100 mm above the ground, aligning its centerline with the top edge of the walls. This allows us to crop the image to a fixed height, ensuring that the walls are always detected at the same height in the image.

### Wiring Diagram

![Wiring Diagram](/media/wiringdiagram.jpg)

### Bill of Materials

| Amount           | Product                                                         | Price in Swiss Francs (CHF 1.00 ~ USD 1.18) |
| ---------------- | --------------------------------------------------------------- | ------------------------------------------- |
| 1                | Raspberry Pi M12 HQ Camera                                      | CHF 45.34                                   |
| 1                | EDATEC 12MP 3.2mm M12 Raspberry                                 | CHF 28.84                                   |
| 1                | Raspberry Pi 4 Model B 8GB                                      | CHF 79.00                                   |
| 1                | Arduino Nano: Multifunktionales Board ATmega328 16Mhz, Mini-USB | CHF 19.95                                   |
| 1                | Tattu LiPo-Akku 14.8V 850mAh 95C 4S1P RL                        | CHF 14.00                                   |
| 1                | L298N Schrittmotorendstufe / H-Brücke / DC Motor Treiber        | CHF 8.90                                    |
| 1                | PDI-5515MG Digital Standard Servo 15.32Kg\*cm                   | CHF 18.90                                   |
| 1                | 12V 251RPM 18Kg\*cm DC Getriebemotor mit Encoder                | CHF 35.90                                   |
| ca. 20 pieces    | M2.5 Screws and nuts                                            | CHF 2.00                                    |
| ca. 10 pieces    | M2 Screws and nuts                                              | CHF 1.00                                    |
| ca. 300g         | 3D printing filament                                            | CHF 5.00                                    |
| ca 20 pieces     | Jumpercabel                                                     | CHF 4.00                                    |
| See stud.io file | LEGO technic bricks                                             |                                             |
| 4                | LEGO technic wheels                                             |                                             |
| **TOTAL**        |                                                                 | **CHF 262.83**                              |

<!-- Obstacle management discussion should include the strategy for the vehicle to negotiate the obstacle course for all the challenges. This could include flow diagrams, pseudo code and source code with detailed comments. -->

All files for the 3D printed parts can be found in the [3D-Printed-Parts](/cad/3d) folder. All parts can be printed without supports at 0.2mm layer height. we recommend using PET-G or nGen for the parts (PLA can also be used).
The wiring diagram can be found in the [Wiring Diagram](/media/wiringdiagram.jpg) file. The instructions for the LEGO chassis can be found in the [stud.io file](/cad/future_eng_flawfactory_v2.io).

## Obstacle Management

### Opening Race

The robot's behavior is modeled using a behavior tree. The [StateMachine](/raspy/statemachine.py) class manages the current state and handles transitions to new states. States can also be scheduled to start in the future, which is useful for delaying turns to avoid hitting inner walls. In the opening race, we use the states “STARTING,” “PD-CENTER,” “TURNING-L/R,” and “DONE.”

We begin by determining the round direction. First, we crop the top half of the image and filter out all black pixels using OpenCV's `cv2.inRange` function. On this Boolean map of black pixels, we detect edges using `cv2.Canny`. By applying `np.argmax`, we find the heights of the walls in pixels at each x-coordinate in the image. The discrete differences between these heights are calculated using `np.diff`, raised to the fourth power, and summed up. This helps us identify the jump in wall height when the inner wall first appears.

For driving, we employ a PD-Controller. The input is derived from the black portion of a region-of-interest extracted from the outer edges of the black-and-white Boolean image. This is compared to a pre-calibrated fixed portion. We follow only the outer wall to avoid collisions with the inner walls, especially if their gap distance is randomized to be small.

Using a small region of interest in the center of the camera feed, we detect the blue and orange lines on the game mat. The color image is converted to HSV for this purpose. Upon encountering such a line, depending on its color, we initiate a turn and decrement the remaining corners counter, allowing us to accurately stop at the end of the round.

![Wall Detection](/media/walls.jpg)

The red outlines show the region of interest used for wall detection.

### Obstacle Race

In addition to extracting a black-and-white image, we convert the cropped color image to HSV. This conversion allows us to more easily and robustly extract red and green pixels. We then use `cv2.Canny` and `cv2.findContours` to search for contours in this image. The centroids of the contours are extracted and stored along with their width and height. The behavior tree is updated with two new states: "TRACKING-PILLAR" and "AVOIDING-PILLAR-R/G".

When handling the HSV color space, special care is needed for colors near the red hue due to the wrap-around effect. The hue value for red is around 0° and 360°, meaning it wraps around the HSV color wheel. To accurately detect red, we create two separate masks: one for the lower range (e.g., 0° to 10°) and another for the upper range (e.g., 350° to 360°). These masks are then combined to form a single mask that accurately captures all red hues. This approach ensures that all shades of red are detected, avoiding issues caused by the hue value wrapping around the color wheel.

![Wall Detection](/media/pillars.jpg)

In the image above you can see the robot detecting the red and green pillars. After processing the image, the program returns a list of found pillars, sorted by their distance to the robot. The robot then drives towards the closest pillar, until it is close enough to the pillar to avoid it. The robot then drives around the pillar and continues to the next one. You can also see the center ROI used for detecting turn marking lines.

<!-- Insert image with behavior tree here -->

![Behavior Tree](/media/behaviors.jpeg)

<!-- Pictures of the team and robot must be provided. The pictures of the robot must cover all sides of the robot, must be clear, in focus and show aspects of the mobility, power and sense, and obstacle management. Reference in the discussion sections 1, 2 and 3 can be made to these pictures. Team photo is necessary for judges to relate and identify the team during the local and international competitions. -->

## Photos

| ![Front](/media/front.jpg) | ![Back](/media/back.jpg)     |
| -------------------------- | ---------------------------- |
| ![Left](/media/left.jpg)   | ![Right](/media/right.jpg)   |
| ![Top](/media/top.jpg)     | ![Bottom](/media/bottom.jpg) |

![Team](/media/Teamfoto%20offiziell.JPG)
![Team](/media/Teamfoto%20lustig.JPG)

**The national menu of Switzerland is fondue. While usually fondue is made from cheese, the team is enjoying a robot fondue 😎🤪**

<!-- The performance videos must demonstrate the performance of the vehicle from start to finish for each challenge. The videos could include an overlay of commentary, titles or animations. The video could also include aspects of section 1, 2 or 3 -->

## Videos

<!-- https://michaelcurrin.github.io/badge-generator/#/generic -->

[![YouTube - Opening Race](https://img.shields.io/badge/YouTube-▶️%20Opening_Race-df3e3e?logo=youtube)](https://www.youtube.com/watch?v=waIHJP2l4eQ)

[![YouTube - Obstacle Race](https://img.shields.io/badge/YouTube-▶️%20Obstacle_Race-df3e3e?logo=youtube)](https://www.youtube.com/watch?v=hp4pJUoMnfw)

## Enabling Reproducibility

To enable the reproduction of our robot, we provide the following installation instructions:

1. Install rapsberry pi os on your raspberry pi using the [official guide](https://www.raspberrypi.org/documentation/installation/installing-images/README.md)
2. After booting up the raspberry pi, connect via ssh, and install the following packages:

```bash
sudo apt-get update
sudo apt-get install python3-opencv python3-websockets python3-numpy python3-pyserial
```

3. Enable the camera using `sudo raspi-config` and reboot the raspberry pi for the changes to take effect. Install the corresponding python module:

```bash
sudo apt-get installpython3-picamera2
```

4. Clone the repository and run the main script:

```bash
git clone https://github.com/robofactory-ch/flawfactory-future-engineers-brescia.git
```

5. Running the robot in dev mode

Check in the config file, if the correct usb port is set for the arduino. Check the correct port with `ls /dev/tty*` and look for the port that is connected to the arduino. Change the port in the config file to the correct port.

Make sure pillars are enabled/disabled in the config file, and that no fixed round direction is set.

Navigate to the `raspy` directory and run the main script:

```bash
cd flawfactory-future-engineers-brescia/raspy
python3 roi.py
```

To launch the robot, open the web interface in your browser and start the robot by clicking the connect button. The robot will now start driving autonomously. To stop the robot, you can close the web interface, press the stop button on the robot or press `ctrl+c` in the ssh session.
