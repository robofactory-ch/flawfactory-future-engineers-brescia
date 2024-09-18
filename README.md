# FlawFactory

**This is the github repository for team FlawFactory for WRO 2024 Brescia. You'll find our documentation in this readme**

## Contents

- [Mobility and Hardware Design](#Mobility-Management)
- [Power and Sense Management](#Power-and-Sense-Management)
  - [Power Management](#Power-Management)
  - [Sense Management](#Sense-Management)

<!-- Mobility management discussion should cover how the vehicle movements are managed. What motors are selected, how they are selected and implemented.
A brief discussion regarding the vehicle chassis design /selection can be provided as well as the mounting of all components to the vehicle chassis/structure. The discussion may include engineering principles such as speed, torque, power etc. usage. Building or assembly instructions can be provided together with 3D CAD files to 3D print parts. -->

## Mobility Management

The robot is built on a Lego chassis, which is supplemented where necessary with custom-designed 3D-printed components. These parts securely attach the motors and electronics to the chassis. The Lego structure incorporates an Ackermann steering mechanism for all four wheels, allowing the robot to navigate tight turns with ease. To ensure even distribution of rotational power, we integrated a differential gear into each axle, powering all four wheels.

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

![Wiring Diagram]()

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
