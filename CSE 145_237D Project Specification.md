### **CSE 145/237D: Triton Droids**

# Tactile Hands

---

**PROJECT CHARTER**

### **Project Overview**

The goal of this project is to develop a low-cost robotic hand prototype with force-sensing resistors (FSRs) on the fingertips of each finger, capable of grasping and releasing simple objects while actively avoiding crushing them. The sensors will be accurate to within 1N. If time allows, additional input methods such as vision-based finger pose estimation, VR teleoperation, and sim-to-real transfer will be explored to extend the system's capabilities. By the end of the quarter, we will demonstrate a working prototype that can perform basic grasping tasks on a small set of test objects, with the force feedback loop actively modulating grip strength. This project focuses on building a reliable tactile grasping foundation while leaving room for future improvements in teleoperation, simulation, and dexterous manipulation.

### **Project Approach**

The approach for this project is to execute the work in multiple phases. As mentioned in the Project Overview, the main objective is to create a robotic hand prototype with calibrated force sensing that can perform basic grasping tasks without damaging objects.

#### **Phase 1 — System Design and Component Selection (Weeks 2–3)**

In this phase, we will finalize the hand platform (based on the AmazingHand design), select force-sensing resistors, identify the microcontroller (ESP32), and define the electronics architecture. Key deliverables include the FSR voltage divider circuit design, the prototype circuit board layout for mounting on the back of the hand, and the selection of calibration weights for the force sensors. We will also set up the simulation environment so that software development can begin alongside the hardware assembly. So, in this phase, we hope to create an initial setup and design decisions needed for integrating the sensors with the robotic hand.

#### **Phase 2 — Hardware Build and Sensor Integration (Weeks 3–5)**

In this phase, we will finish assembling the mechanical hand, fabricate and populate the prototype sensor board, and attach the FSRs onto the fingertips. The ESP32 will be programmed to read force sensor values over the voltage divider network and transmit them over USB serial. Initially, fake force values will be used to validate the data pipeline end-to-end before real sensors are connected. Force sensor calibration will run in parallel, using 3D-printed weights to map raw readings to Newtons with accuracy within 1N. The simulation of the AmazingHand will also be brought up during this phase.

#### **Phase 3 — Testing, Calibration Validation, and Basic Grasping (Weeks 5–6)**

In this phase, we will test the hand's ability to open and close reliably, validate the force sensor calibration against known loads, and demonstrate a basic grasp-and-release cycle on at least one simple object. The force feedback loop will be tested to confirm the system is aware of its grasp strength and actively avoids crushing the object. Any mechanical or electrical issues found during testing will be iterated on. This phase culminates in the MVP demonstration.

#### **Phase 4 — Reach Goals and Documentation (Weeks 7–10)**

After the MVP is validated, the team will pursue optimizations and some potential reach goals. These include VR-based teleoperation of the hand using a Quest 2 headset, vision-based finger pose estimation from an iPhone or webcam, sim-to-real object rotation, and mounting the hand on a robotic arm. Final documentation, demonstration gms, and the project report will be completed during this phase.

Overall, the robotic hand system will:

* Be a robotic hand prototype with FSR-based tactile sensing on each fingertip  
* Include a custom prototype circuit board with voltage dividers, an ESP32, and vertical headers for sensor connections  
* Demonstrate calibrated force measurement accurate to within 1N  
* Perform controlled grasp and release of simple objects  
* Use force feedback to actively avoid crushing objects

### **Minimum Viable Product**

The minimum viable product is a robotic hand prototype with force-sensing resistors on the tips of each finger, calibrated to within 1N accuracy, that can demonstrate controlled finger motion and the ability to pick up at least one simple object. The force information must be clearly used to avoid crushing the object.

#### **MVP Goals:**

1. #### **An operational robotic hand prototype with tactile sensing**

   * The fingers can open and close in a controlled way  
   * FSRs are mounted on each fingertip and connected to the prototype board  
   * The ESP32 reads force sensor values using voltage dividers and transmits force readings over USB  
   * The structure is mechanically stable during operation

2. #### **Calibrated force sensing accurate to within 1N**

   * 3D-printed calibration weights are used to map raw ADC readings to force in Newtons  
   * The non-linear relationship will be captured using a polynomial function  
   * The calibration procedure is documented and repeatable for all fingers

3. #### **Force-aware grasping demonstration**

   * The hand can grasp at least one simple object  
   * The system monitors fingertip force in real time during the grasp  
   * The hand actively adjusts grip strength to avoid crushing the object

4. #### **A repeatable test setup**

   * We can demonstrate the force-aware grasping behavior multiple times  
   * Results are documented with video, force logs, and test notes

#### **Milestones for MVP:**

* Test hand opening/closing motion  
* Demonstrate successful grasp and release of a simple object  
* Validate force sensor accuracy to within 1N  
* Attach force sensors, microcontroller, and electronics to read force sensors  
* Show that the system is aware of its grasp strength and actively avoids crushing the object

We can achieve this MVP by focusing on the core sensing and grasping pipeline first: get the hand moving, get the sensors reading, calibrate them, and close the loop. Advanced features like teleoperation, vision, and simulation are deferred until after the MVP is validated.

### **Reach Goals After MVP**

After establishing the MVP, the team will pursue additional features to improve the system. These stretch goals will only be explored after the core force-aware grasping pipeline is complete.

1. **VR Teleoperation** — Use a Quest 2 headset with hand position tracking to teleoperate the robotic hand.  
2. **Vision-Based Finger Pose Estimation** — Use an iPhone app (or laptop webcam as fallback) for finger pose estimation. This can act as a fallback to VR tracking.  
3. **Sim-to-Real Object Rotation** — Demonstrate object rotation in the hand of a simple object, first in simulation and then transferred to the real hardware.  
4. **Robotic Arm Integration** — Attach the robotic hand onto a robotic arm and perform basic hard-coded arm joint control for basic pick-and-place.  
5. **VR Arm Teleoperation** — Use the VR headset's hand position tracking to control the robotic arm's end-effector position for teleoperated pick-and-place.

### **Technological Requirements**

Hardware:

* **ESP32-S3 DevKit** (WROOM-1 module) as the main microcontroller, chosen for its 12-bit ADC, native USB, and ample GPIO for four FSR channels plus servo control.  
* **Four Pololu Force-Sensing Resistors** \#2728 mounted to the fingertips of the AmazingHand.  
* **AmazingHand** mechanical platform with four Feetech SCS0009 micro servos driving the fingers.  
* Power: **2S LiPo** (7.4V nominal) stepped down to 5V using a buck converter, with a Schottky diode to prevent USB backfeed during bench debugging.  
* **Prototype board**: solderable breadboard mounted on the back of the hand, hosting the ESP32-S3, four voltage dividers (22kΩ reference resistors), decoupling capacitors (100nF and 10µF near ADC inputs), and vertical 0.1" headers for FSR Dupont connections.  
* **Reference loads**: 3D-printed PLA calibration weights at 0.5N, 1N, 2N.  
* **Reference objects** for grasping tests: a plastic cup and additional simple objects to be selected during testing.

Software:

* Firmware: **Arduino core** for ESP32, ADC sampling at 100Hz, USB CDC serial output at 115200 baud, CSV format of timestamp plus four force values in Newtons.  
* Simulation: **MuJoCo or Isaac Sim**.  
* Host: **ROS2 on Ubuntu**, with a serial bridge node publishing force readings on /tactile/forces and subscribing to /servo/commands.  
* VR: **Unity** 2022.3 LTS building the Quest 2 APK from leggedrobotics/unity\_ros\_teleoperation.  
* Vision: MediaPipe Hands for webcam fallback, ARKit HandPose for the iPhone.

Tooling:

* UCSD Makerspace for fabrication and assembly.  
* Bambu Lab A1 Mini for 3D-printed parts and calibration weights.  
* Bench multimeter and oscilloscope.

### **Testing Infrastructure**

Testing is supported by dedicated fixtures and a standardized data pipeline:

* Calibration rig: a fixture holds each sensor flat and level. Calibration weights (0.5N, 1N, 2N, 4N) are stacked on the FSR in sequence, with ADC readings logged over serial for each weight. Each sensor is cycled five times to characterize repeatability.  
* Reference test objects: a cup and additional simple objects selected during testing. Each object has a known crush threshold used to define pass/fail for the grasp test.  
* Success criteria:  
  * Error under 1N across the force range.  
  * Grasping: 5 of 5 consecutive grasp-release cycles on each reference object without exceeding the crush threshold.

### **Constraints, Risk, and Feasibility**

There are certain constraints and risks when implementing this project:

1. **Force sensor accuracy may be difficult to achieve within 1N**  
   * FSRs are nonlinear and can change with temperature and repeated loading  
   * To reduce this risk, we will use 3D-printed calibration weights with known masses and develop a calibration curve that accounts for nonlinearity  
   * We will also test sensor repeatability over multiple loading cycles  
2. **The prototype PCB and wiring may introduce noise into force readings**  
   * Analog signals from FSRs are sensitive to wiring length, grounding, and voltage reference stability  
   * To reduce this risk, we will use proper voltage divider sizing, keep analog traces short, and add decoupling capacitors near the ESP32's ADC inputs  
3. **The hand may not produce enough force for reliable grasping**  
   * To reduce this risk, we will start with simple, lightweight objects and a basic grasp geometry  
4. **Reach goals depend on multiple independent subsystems working together**  
   * VR teleoperation requires the Quest 2 APK and ROS integration  
   * Vision-based pose estimation requires either an iPhone app or a webcam pipeline, plus a mapping from finger pose to servo commands  
   * To reduce this risk, we will develop reach goal subsystems in parallel and only integrate them after the MVP is validated  
   * Each reach goal is assigned to a specific team member to maintain focus  
5. **Limited time and available hardware**  
   * We will use available parts and tools whenever possible  
   * The MVP will remain the main priority throughout the quarter  
   * We will avoid depending on non-essential parts for the MVP

---

**GROUP MANAGEMENT**

### **Communication**

Bryce will coordinate the project by organizing tasks, tracking progress, and keeping the group on schedule. Development decisions will primarily be made by consensus so that all members can contribute to design choices and tradeoffs. If the group needs to move forward quickly, Bryce will make final calls as project lead.

The team will communicate through iMessage and Discord for fast updates and use weekly team meetings on Wednesdays to review progress, plan upcoming work, and raise any blocking issues for discussion. Schedule management and task ownership are defined in the Project Milestones and Schedule section below.

### **Decision-making Process**

Decisions are handled at three levels:

1. Subsystem decisions are made by the subsystem owner. The owner is expected to document notable decisions.  
2. Cross-subsystem decisions are discussed in the weekly Wednesday meeting. If consensus is not reached by the end of the meeting, the most affected subsystem owner makes the final call and Bryce is informed.  
3. Scope and schedule decisions require Bryce's sign-off as project lead. If MVP completion is at risk by Week 4, reach goals are cut.

Blockers raised in Discord are sent to Bryce within 24 hours, either resolved directly or routed to the correct owner with a deadline.

### **Documentation Process**

Documentation is written continuously, not at the end of the quarter. Each subsystem owner is responsible for their own docs:

* Code and hardware live in a shared GitHub repo under the Triton Droids organization, with a top-level README and a per-subsystem subdirectory (firmware, board, calibration, simulation, vr, vision).  
* The calibration procedure is written up as a standalone Markdown document in the repo as Shree completes it, including the rig photo, weight set, procedure steps, and expected outputs.  
* Design decisions and weekly progress are logged in a shared Google Doc, with one entry per Wednesday meeting summarizing what shipped, what slipped, and what is next.  
* CAD files for the prototype board case (mounted on the back of the hand) are version-controlled in the repo alongside the mechanical files.  
* Final deliverables at end of quarter: project report (PDF) and an annotated wiring diagram. The repo itself serves as the primary reference for reproducing the system.

---

**PROJECT DEVELOPMENT**

### **Development Roles**

* **Mechanical/Electrical Prototyping and Sensor Integration**  
  * Finish the hand, design and populate the prototype sensor board, add dupont connectors for FSRs, make voltage divider circuits  
* **Force Sensor Calibration**  
  * Develop the calibration procedure, create reference loads, map ADC readings to Newtons, validate accuracy  
* **Simulation**  
  * Bring up the AmazingHand in simulation (MuJoCo or Isaac Sim), validate kinematics, prepare for sim-to-real  
* **VR Teleoperation**  
  * Build and deploy the Quest 2 APK from the unity\_ros\_teleoperation framework, establish ROS2 communication  
* **Vision-Based Finger Pose Estimation**  
  * Develop an iPhone app or webcam pipeline for finger pose tracking, output normalized finger curl values  
* **Actuation and Control**  
  * Integrate servo control with force feedback, implement the force-aware grasp controller  
* **Testing, Integration, and Documentation**  
  * Test the hand on simple objects, validate the force feedback loop, record progress and results

### **Role Assignment**

* **Sidath**: VR teleoperation — build the Quest 2 APK from the unity\_ros\_teleoperation repo, establish the ROS communication link. Also supports mechanical/electrical prototyping, actuation, testing, and integration.  
* **Thomas**: Vision-based finger pose estimation — develop an iPhone app (preferred) or laptop webcam pipeline that outputs a 4-element list of normalized floats \[0.0–1.0\] representing the curl of each finger. Research hand pose estimation frameworks (ARKit, MediaPipe) and implement the mapping from finger joint angles to a single normalized value per finger.  
* **Bryce**: Electronics and sensor integration — design and build the prototype PCB that mounts on the back of the hand. The board includes an ESP32, voltage dividers for the FSRs, and vertical pin headers for sensor connections. Develop the firmware to read force sensor values and transmit them over USB serial (initially using fake values for pipeline validation). 3D print calibration weights for force sensor characterization. Project lead responsibilities: schedule tracking, logistics, milestone coordination.  
* **Shree**: Force sensor calibration — develop and document the calibration procedure for the FSRs. Use the 3D-printed calibration weights to apply known forces, record ADC readings, and fit a calibration curve. Validate that the calibrated output is accurate to within 1N across the relevant force range. Characterize sensor repeatability and drift.  
* **Ali**: Simulation — get the AmazingHand working in a physics simulation environment (MuJoCo, Isaac Sim, or PyBullet). Set up the URDF/MJCF model, verify finger kinematics, and prepare the simulation for future sim-to-real transfer experiments.

### **Technical Assignment**

* **Ali:** Worked at the Hao Su Lab at UCSD in summer 2025 on a robotic hand research project accepted to ICLR 2026\. Built and tested simulation tasks in Isaac Lab and MuJoCo for catching, lifting, grasping, and in-hand reorientation, designed reward functions, programmatically generated 600+ hand variants, and ran 1,000+ PPO training experiments. Experience with URDF and adding grippers to robot arms in Isaac Sim. This maps directly to getting the hand set up in simulation and developing the sim-to-real pipeline for this project.

## **Tactile Hands – Project Timeline**

**MVP by Week 6**

The timeline in the Project Milestones and Schedule section is the source of truth for week-by-week ownership. Bryce maintains it and updates it after each Wednesday meeting. Every owner reports whether their current-week tasks are on track, at risk, or slipped. Any task that slips more than one week triggers a replan discussion, and Bryce communicates the updated timeline to the group within 48 hours.

| Task | W2 | W3 | W4 | W5 | W6 | W7 | W8 | W9 | W10 |
| ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- | ----- |
| System Design & Component Selection | X | X |  |  |  |  |  |  |  |
| PCB Design & Fabrication (Bryce) |  | X | X |  |  |  |  |  |  |
| FSR Calibration Procedure (Shree) |  |  | X | X | X |  |  |  |  |
| Hand Assembly & Sensor Integration |  | X | X | X |  |  |  |  |  |
| Firmware Dev — ESP32 USB Serial (Bryce) |  |  | X | X |  |  |  |  |  |
| Simulation Bring-Up (Ali) |  | X | X | X |  |  |  |  |  |
| Testing & Force-Aware Grasping |  |  |  | X | X |  |  |  |  |
| MVP Completion & Validation |  |  |  |  | X |  |  |  |  |
| Quest 2 VR Teleoperation APK (Sidath) |  |  |  |  |  | X | X | X |  |
| iPhone/Webcam Finger Pose (Thomas) |  |  |  |  |  | X | X | X |  |
| Sim-to-Real Transfer (Ali) |  |  |  |  |  |  | X | X |  |
| Arm Integration (if time) |  |  |  |  |  |  |  | X | X |
| Final Refinement & Documentation |  |  |  |  |  |  |  | X | X |

