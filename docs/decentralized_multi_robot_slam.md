# Decentralized Multi-Robot SLAM with slam_toolbox

*decentralized_multirobot_slam_toolbox_node* extends slam_toolbox for multi-robot mapping and localization. This document provides details on the following:

1. Getting started guide (Multi-Robot SLAM Simulation)
2. Multi-Robot Mapping General Introduction
3. Decentralized Multi-Robot SLAM
4. Topics & Shared data
5. Setting up Decentralized multi-robot slam_toolbox for your robot fleet

> TL;DR: Each robot runs its own slam_toolbox instance; These slam_toolbox instances communicate with each other through `LocalizedLaserScan` messages; each slam_toolbox instance generates a global map of the environment. **Note:** Starting positions of all robots should be known and a global odometry frame is required.

---

## 1. Getting Started — Multi-Robot SLAM Simulation
The [slam_toolbox_multi_robot_demo](https://github.com/acachathuranga/slam_toolbox_multi_robot_demo.git) package provides a quick and easy way to get started with decentralized multi-robot SLAM. Clone and compile the package following the instructions on package repository.

This package launches a simulation with two robots, each running a namespaced instance of `decentralized_multirobot_slam_toolbox_node`. A shared global odometry frame (`/global_odom`) is established using static transforms, and the robots exchange localized scan data to maintain a consistent global map.

![multirobot_slam_simulation](../images/decentralized_multirobot/simulation.gif?raw=true "slam_toolbox_multi_robot_demo")

### Launch the Simulation
```bash
ros2 launch slam_toolbox_multi_robot_demo multi_tb3_simulation_launch.py
```
In two seperate terminals, run *teleop_twist_keyboard* to control the robots. The merged map will appear on both rviz windows when moving the robots around in the environment.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard __ns:=/robot1 --ros-args -p stamped:=true
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard __ns:=/robot2 --ros-args -p stamped:=true
```

## 2) Multi-Robot Mapping General Introduction

### What is Multi-Robot Mapping?

Multi-robot mapping refers to the process of multiple robots performing SLAM (Simultaneous Localization and Mapping) at the same time within a shared environment. Each robot explores a portion of the space and builds its own local map; these individual maps are then aligned or merged into a **global map** representing the entire environment.

This approach allows robots to cover larger areas faster than a single robot, improves robustness against individual robot failures, and provides redundancy in sensor coverage.

Typical examples include:
- Warehouses where multiple AMRs build and maintain a unified facility map.
- Search-and-rescue operations where ground and aerial robots explore separate sectors.
- Multi-floor or multi-building deployments where each robot contributes to a common navigation map. (Beyond 2D-SLAM scope)

---

### Why is a Network Connection Needed?

For regular single-robot SLAM, the sensor data collection, processing and map generation are often done within the robot's computer itself. Therefore, for single-robot SLAM, a network connection is often not a requirement. In contrast, for multi-robot SLAM; for robots to produce a shared global map, they must **exchange information**.  These information can range from full pose-graph data, odometry, and raw sensor feeds to higher-level representations such as processed scans or optimized keyframes.

Often for real deployments, communication layer typically uses Wi-Fi networks (mesh or infrastructure-based). Based on the communication architecture, multi-robot SLAM can be classified as centralized or decentralized (AKA 'distributed').

![network_architectures](../images/decentralized_multirobot/network_architectures.png?raw=true "Centralized vs Decentralized")

The above image shows two examples of centralized and decentralized SLAM approaches. The centralized approach shares all laser scans over the network and a single slam_toolbox node subscribes to all laser data and generates the global map. In the decentralized approach, each robot runs their own slam_toolbox node and generates their own global map. 

In multi-robot SLAM architectures that rely on a **centralized** backends:
- All robots stream data to a single server that performs map fusion and graph optimization.
- While easy to manage, it scales poorly and becomes a single point of failure.
- Bandwidth requirements are often high because every robot sends raw or semi-processed data to the central node.

A **decentralized** (or distributed) architecture shifts computation to the edge:
- Each robot performs its own local SLAM independently.
- Robots can exchange only minimal information (e.g., localized scans, key poses) needed to align their maps.
- The system remains functional even if part of the network drops out.


## 3) Decentralized Multi-Robot SLAM
*decentralized_multirobot_slam_toolbox_node* extends slam_toolbox for decentralized multi-robot mapping and localization. Each robot runs a slam_toolbox instance and generates their own map. Each of these slam_toolbox instances momentarily share their pose-graph data with each other to build a global pose-scan graph, and subsequently the global map.

<!-- ![node_graph](../images/decentralized_multirobot/node_graph.png?raw=true "Node Graph" ) -->
<p align="left">
  <img src="../images/decentralized_multirobot/node_graph.png?raw=true"
       alt="Node Graph"
       width="30%">
</p>

In practical deployments, such as logistics warehouses, campus delivery fleets, or large-scale exploration; robots often operate in partially connected networks where:
- Wi-Fi coverage is inconsistent.
- Communication latency varies by several hundred milliseconds.
- Continuous streaming of raw sensor data is infeasible.

Therefore, **multi-robot mapping must be robust to bandwidth limits and intermittent connectivity.**  
This motivates the design of the *decentralized multirobot_slam_toolbox_node*, which achieves collaborative mapping by exchanging lightweight `LocalizedLaserScan` messages rather than full laser topics or pose-graphs.


## 4) Topics & Shared Data

This section explains what data are shared, how frames/headers are written, and why this design keeps bandwidth low and behavior predictable.

### 4.1 Parameters

- **`scan_share_topic`** *(string, default: `/localized_scan`)*  
  Topic used to **publish and subscribe** `LocalizedLaserScan` messages across robots.

---

### 4.2 Published / Subscribed Topics

**Publish** (per robot)
- **`<scan_share_topic>`** (`slam_toolbox::msg::LocalizedLaserScan`)  
  Emitted **after** the local scan is added to the robot’s SLAM pipeline (i.e., *localized*).  
  The message includes:
  - `scan` *(sensor_msgs/LaserScan)* — the original laser scan data (ranges/intensities).
  - `pose` *(geometry_msgs/PoseWithCovarianceStamped)* — the robot pose associated to this scan.
  - `scanner_offset` *(geometry_msgs/TransformStamped)* — Scanner-to-RobotBase static offset on the host robot. 

  **Note:** TF data are not broadcast between robots. This significantly reduces network usage. However, peer robots can no longer query each other’s TF tree directly, which is why each shared `LocalizedLaserScan` message explicitly includes its own scanner-to-base transform.

**Subscribe** (per robot)
- **`<scan_share_topic>`** (`slam_toolbox::msg::LocalizedLaserScan`)  
  On receipt:
  1) Self-originated scans are **ignored** (namespace check).  
  2) The scan is converted and passed to the local mapper via `addExternalScan(...)`.  
  3) If accepted, a TF is published to place the **producer’s** base in the **consumer’s** map frame.

---

### 4.3 Namespaces & Header conventions

Each node derives an **identity** from its own namespace, e.g. `robot1`, `robot2`.  
When a scan is published, this identity is **prefixed** to relevant frames so peers can unambiguously tell *who* produced the data.

Producer-side header rules (simplified):

- `scan.header.frame_id` → `/<producer_ns>/<original_scan_frame>`  
- `pose.header.frame_id` → `/<producer_ns>/<map_frame>` 

Consumer-side self-filter:

- Extract `<producer_namespace>` from scan header in received `LocalizedLaserScan` message
- If `<producer_namespace> == <host_namespace>`, **drop** (ignore self).

> Why this matters: Every peer sees which robot produced a scan by inspecting the frame prefix; no frame collisions in RViz/TF, and no ambiguity when adding inter-robot constraints.

---

### 4.5 Why **localized scans** (not raw scans / full graphs)?

- **Bandwidth**: Raw LiDAR from N robots is prohibitive on real Wi-Fi/mesh. Localized scans are sparse (one per processed scan) and carry **exactly** the signal needed for alignment. No need to share any other additional topics (e.g. TF). 
- **Decoupling**: Each robot retains its own internal graph; peers are never required to understand or mutate each other’s optimizations.  
- **Robustness**: If a message is dropped, peers simply miss one inter-robot constraint; no central backlog or single point of failure.  

---

## 5) Setting up Decentralized multi-robot slam_toolbox for Your Fleet

### 5.1 Prerequisites

- Shared ROS 2 network between all robots (same DDS domain ID)
- Known starting positions for all robots (at least roughly aligned)
- Static transform(s) defining the global reference frame (e.g., `global_odom → robotX/odom`)

---

### 5.2 Global reference frame
`decentralized_multirobot_slam_toolbox_node` requires a common reference odometry frame in order to generate the global map. 

In slam_toolbox parameters, the odometry frame should be set to this global reference odometry frame.
```bash
# ROS Parameters
odom_frame: global_odom
```

Based on their starting locations, each robot needs to publish a static transform from the global odometry frame to robot's individual odometry frame. This can be done by launching a static transform publisher as follows (Example launch file snippet):

```bash
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_tf_broadcaster',
    namespace=<robot_namespace>,
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    arguments=[
        '--x', str(<robot_start_pos_x>),
        '--y', str(<robot_start_pos_y>),
        '--z', str(<robot_start_pos_z>),
        '--roll', str(<robot_start_ori_roll>),
        '--pitch', str(<robot_start_ori_pitch>),
        '--yaw', str(<robot_start_ori_yaw>),
        '--frame-id', 'global_odom',
        '--child-frame-id', 'odom'
    ]
)
```
For more information, refer to [multi_tb3_simulation_launch.py](https://github.com/acachathuranga/slam_toolbox_multi_robot_demo/blob/7c6caeba15ee25cb126815e3f6861577dda7dafa/launch/multi_tb3_simulation_launch.py#L97) launch file in [slam_toolbox_multi_robot_demo](https://github.com/acachathuranga/slam_toolbox_multi_robot_demo.git) package.

> **Note:** The global odometry frame must remain consistent across all robots.
> In simulation this frame can be `world` or `odom`, while in real deployments it can be established via GPS manual initialization or other methods.


---

### 5.3 Launch Configuration

Each robot should launch:
1. Its **own namespaced instance** of `decentralized_multirobot_slam_toolbox_node`
2. A **TF tree** for that robot (e.g., `global_odom → odom → base_link → laser`)
3. The **same scan_share_topic** parameter (default `/localized_scan`). Note that this topic should not be within robot's namespace.

Example:

```bash
ros2 launch slam_toolbox online_async_decentralized_multirobot_launch.py namespace:=robot1
```
Once all nodes are running and exchanging localized scans, each robot maintains its own pose graph but contributes to a consistent shared map — achieving collaborative SLAM without any centralized coordination.


## 6) Limitations

- Currently, this implementation supports **only one Laser Scanner per slam_toolbox instance**.  
  To use multiple laser scanners on the same robot, you must run **multiple slam_toolbox nodes**, in multiple namespaces, with each instance handling a single scanner.
