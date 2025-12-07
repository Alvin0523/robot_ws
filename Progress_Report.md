# **Progress Report: Visual Navigation, Isaac ROS Integration, and Common Core Autonomy Platform (By @Alvin_0523, Wei Ming)**

## **1. Visual Navigation Development Using Isaac ROS**

The initial phase focused on establishing a visual-navigation pipeline. Isaac ROS was recommended as the primary framework, and the goal was to replicate an existing open-source TurtleBot-based implementation. However, the reference code was developed for an older Isaac ROS distribution, whereas the current work uses **Isaac ROS 3.2** on **ROS 2 Humble**.
Because Isaac ROS tightly couples its releases with specific Ubuntu and ROS versions, the legacy example was not directly compatible. Significant portions of the codebase had to be rewritten and adapted to match the updated APIs and package structures. This incompatibility introduced substantial complexity and extended the development timeline, but the issues were eventually resolved.

A known performance limitation also surfaced during testing: **perception–planning latency** on Jetson Orin NX–class hardware. As highlighted in the NVIDIA forums, many developers experience frame-processing delays on Orin NX and lower-tier Jetsons due to limited compute throughput. Frame desynchronization directly affects the quality of path planning.
Despite this, the proof-of-concept (PoC) was successful—**Isaac ROS visual navigation with Nav2 was demonstrated on TurtleBot**, establishing feasibility.

---

## **2. Transition to Pixhawk as a Universal MCU Platform**

Given Q Team’s mandate for rapid deployment and prototyping, repeatedly learning new microcontroller environments is inefficient. A **universal, high-capability MCU** was required—one that supports ground, aerial, surface, and underwater platforms with minimal reconfiguration.
**Pixhawk** was selected for this purpose due to:

* Mature Autopilot firmware (ArduPilot/PX4)
* Broad multi-domain vehicle support
* Robust tuning and configuration ecosystem
* Standardized interfaces suitable for common-core architectures

The PoC was achieved once reliable **DDS communication between Pixhawk and Jetson** was established over ROS 2. With that foundation, Nav2 was integrated and validated. When the RealSense-based VSLAM was stable and the transform tree was consistent, **the robot achieved autonomous navigation**, confirming the feasibility of a **Pixhawk–Jetson AMR Common Core**.

---

## **3. Current Robot Platform Limitations**

The prototype platform demonstrates functional autonomy but is **not deployment-ready**. Key limitations include:

* **Insufficient power system**

  * Battery capacity is inadequate for operational runtime
  * Absence of a proper Power Delivery Module (PDU) for safe distribution and protection

* **Temporary mechanical structure**

  * The chassis is not designed around the AGX Orin’s size or thermal requirements
  * Mounting constraints reduce maintainability and safety

* **Inaccurate odometry affecting localisation**

  * RealSense D435i has a ~30 cm blind spot
  * Low mounting height reduces geometric features for VSLAM
  * Resulting drift undermines consistent path planning

Mitigation options include:

* Fusion of **wheel encoder odometry**
* Integration of **additional RealSense units**
* **LiDAR–VSLAM fusion**
* Combining all modalities for maximum robustness

These improvements will be required before the platform can be considered suitable for deployment.

---

## **4. Agentic Interaction and Higher-Level Autonomy**

A basic agentic interface was developed using Whisper for speech-to-text and ROS topic mapping for direct command execution.
For more sophisticated interaction—task decomposition, tool invocation, contextual reasoning—a **ROS Agentic System (ROSA)** integrated with LangChain will be required.
Scene understanding can be enhanced using **Vision-Language Models (VLMs)** to interpret environment context and support decision-making.

---

## **5. Simulation Considerations**

Isaac Sim provides a high-fidelity environment and is ideal for advanced testing of:

* Autonomous behaviors
* Physics-based interactions
* Agentic systems
* GPU-accelerated perception pipelines

However, due to the **rapidly evolving physical design** of the PoC robot, maintaining a fully accurate simulation model is impractical at this stage.
Only preliminary URDF import tests were performed; no full simulation workflow was built.

recommendations:

* Use **Gazebo** for quick testing of basic mobility and path planning
* After hardware stabilisation post-PoC, invest in **Isaac Sim** for tuning navigation, autonomy stacks, and agentic behaviors

The Nav2 + Isaac ROS stack runs flawlessly in the canonical simulation environment because NVIDIA’s examples are built for **Nova Carter**, an off-the-shelf platform with well-defined sensors and geometry. Our custom platform differs significantly, so simulated results do not directly translate to deployment.

---

## **6. Summary**

This phase successfully delivered multiple proof-of-concept validations:

* Isaac ROS visual navigation on real hardware
* Pixhawk–Jetson DDS communication
* Stable Nav2 autonomous navigation
* Whisper-based voice interaction prototype

The work confirms that a **Common Core architecture for autonomous navigation**—centered on Jetson + Pixhawk—is feasible for Q Team’s rapid prototyping needs.

However, substantial engineering is required to transform the PoC into a deployable system, especially in power delivery, mechanical integration, and robust multi-sensor localisation.
