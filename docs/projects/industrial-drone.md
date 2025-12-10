## The Industrial Drone

<div align="center">
  <img src="../../images/drone/industrial_drone_cropped.jpeg" width="700" />
  <p><em> </em></p>
</div>

I’ve always wanted to build my own drone, so in 2023 I finally decided to make it happen. A drone is the perfect project for me because it combines all of my interests: power electronics, digital electroncis, RF communication, mechanics, embedded software. I chose to build a larger 'industrial-style' platform to serve as a platform for my own future custom attachments and experiments.

While the project used mostly commercial components, the system integration was a challenge. Work began in late 2023 when a mountain of parts arrived without instructions. To turn them into a functional drone, I designed and 3D printed custom mounts for nearly every subsystem—including cameras, batteries, antennas, and power distribution electronics.

**Key learnings:**
  
  - ArduPilot flight controller configuration.
  - Drone systems: Basic components, battery technologies, wireless communication and protocols (MavLink, ELRS), and control electronics.
  - Practical electronics skills.
  - Deep dive on 3D printing (PLA, PETG) and 3D modelling (SolidWorks, OnShape).

**Future of the project:** 
  
  - Design of my own power distribution board that measures current, filters the power and regualtes it for the electronics.
  - Developing my own gimbal / camera orientation control system for the drone. 

## Project Details

The project began with system simulation using eCalc to define the optimal powertrain. The motor, propeller, and ESC combination was selected to maximize payload capacity while ensuring high efficiency for extended flight times. Simulations indicated that an 11x4.5 propeller configuration offered the best performance. A 4S battery architecture was chosen to minimize overall system weight and costs. The final design has, accoring to the simulations, a theoretical thrust-to-weight ratio of 3:1 and a hover flight time of approximately 24 minutes, providing a lifting capacity of 3–4 kg with ample margin for future accessories.

<div align="center">
  <img src="../../images/drone/simulations.png" width="750" />
  <p><em> Simulation results for the components. </em></p>
</div>

**Specs:**
  
  - Weight: ~2.2 - 2.5 kg (With 9200 mAh 4S LiPo battery)
  - Motor Power: 4x 800W -> ~3-4 kW peak power
  - Estimated Max Thrust: ~ 4 kg
  - Flight Time: 10 minutes rough flight, Smooth flight +20 minutes
  - Max Current:  +120 A (measured, from 4S battery)

**Components:**
  
  - Motors: BrotherHobby Avenger V3 2812-900KV
  - ESC: Hobbywing XRotor Pro 40A (60A Peak)
  - Power Distribution Board: Mateksys XCLASS PDB FCHUB-12S
  - Propellers: Biblade 11 x 4.5, carbon reinforced / 10x5 Triblade
  - Flight Controller: Pixhawk 2.4.8
  - Controller communication: Radiomaster RP3 ELRS transceiver with two antennas
  - Frame: F450 Carbon Fiber with retracting arms
  - VTX: Walksnail Avatar HD Pro kit V2

The frame is a generic carbon fiber kit which arrived with no mounts for the battery, GoPro, or other peripherals. Starting with just the bare frame and countless bags of parts, I had to design custom mounts for practically every separate component to create a the complete drone system.

<div align="center">
  <img src="../../images/drone/frame.jpg" width="700" />
  <p><em> The F450 carbon fiber frame. </em></p>
</div>

### Battery Mount and Power Distribution

Since this drone is quite large and uses large batteries (~10 000 mAh / 4s) I used a separate RC car battery holder that was meant for heavy batteries. It was installed upside down on the drone with custom 3D printed mounts. The drone has a current sensor which I designed to lay in between the battery mount and the frame. This saved space and led to a convenient position for the battery XT-connectors.

<div align="center">
  <img src="../../images/drone/battery_mount.jpg" width="700" />
  <p><em> </em></p>
</div>

<div align="center">
  <img src="../../images/drone/battery_mount_2.jpg" width="700" />
  <p><em> </em></p>
</div>

The current sensor feeds into the Power Distribution Unit (PDU), which splits power to the ESCs and regulates voltage for the flight controller and peripherals. I selected the Mateksys FCHUB-12S for its high current rating and control features, allowing me to toggle the video transmitter (VTX) via the remote controller to prevent overheating while on the ground.

<div align="center">
  <img src="../../images/drone/PDU.jpg" width="700" />
  <p><em> </em></p>
</div>

### Wiring and Cable Management

Reliability was a key priority for this build. I focused heavily on clean soldering (with flux) and cable management; almost every wire is neatly sheathed and routed to prevent snagging or vibration damage. The drone is designed to be modular and maintenance-friendly. High-quality XT and bullet connectors are used throughout, allowing components to be easily disassembled for upgrades or repairs.

<div align="center">
  <img src="../../images/drone/ESCs.jpg" width="700" />
  <p><em> </em></p>
</div>

<div align="center">
  <img src="../../images/drone/braided_calbes.jpg" width="700" />
  <p><em> </em></p>
</div>

### Accessory Mounts

To make the platform versatile, I designed a modular accessory mount system. It features a 45x45mm mounting holes, allowing virtually any custom device—from cameras to sensors—to be attached securely. In the future I am interested in desining a camera gimbal that fits on the accessory mount system.

Currently, it can host a GoPro mount, a separate flashlight mount and an adjustable FPV camera mount. I also designed a specific holder for the FPV video transmitter to ensure it sits in the prop wash for cooling. The dual antennas are positioned in a V-formation at approximately 50 degrees to maximize video link range and signal diversity.

<div align="center">
  <img src="../../images/drone/camera_mount.jpg" width="700" />
  <p><em> </em></p>
</div>

<div align="center">
  <img src="../../images/drone/gopro_mount.jpg" width="700" />
  <p><em> </em></p>
</div>



The drone uses the ELRS protocol with the controller. I chose the Radiomaster RP3 ELRS transceiver with two antennas. A custom mount was also designed and 3D printed for this. The drone also has Telemetry link over MavLink and the drone or its status can be controlled from a laptop. The drone is also equipped with GPS and suits for completely automated missions.

<div align="center">
  <img src="../../images/drone/ELRS_mount.jpg" width="700" />
  <p><em> </em></p>
</div>

### Covers and other parts

I also designed a cover for the flight controller and its cables and improve the aerodynamics of the drone. The 3D printer calibration was successful, and this is one of the best PETG prints I have ever made. Later, I redesigned the entire cover and added a lid with an OLED display showing the drone's status.

<div align="center">
  <img src="../../images/drone/cover_v1.jpg" width="700" />
  <p><em> </em></p>
</div>

<div align="center">
  <img src="../../images/drone/first_tests.jpg" width="700" />
  <p><em> </em></p>
</div>

<div align="center">
  <img src="../../images/drone/oled_cover.jpg" width="700" />
  <p><em> </em></p>
</div>

### Finished Build

The drone is practically ready and airworthy. The drone flies nicely and works as expected. Overall the project was a success and a huge learning process for me. The project will likely continue according to the plans mentioned above.












