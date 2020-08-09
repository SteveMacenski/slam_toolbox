---
title: 'SLAM Toolbox: SLAM for the dynamic world'
tags:
  - ROS
  - ROS2
  - robotics
  - SLAM
  - Lidar
authors:
  - name: Steve Macenski
    orcid: 0000-0003-1090-7733
    affiliation: "1"
  - name: Ivona Jambrecic
    affiliation: "2"
affiliations:
 - name: Open-Source Robotics Engineering Lead, Samsung Research
   index: 1
 - name: Software Engineering Intern, Simbe Robotics
   index: 2
date: 31 July 2020
bibliography: paper.bib

---

# Summary

Developments in the field of mobile robotics and autonomous driving have resulted in the use of robots and vehicles in retail stores, hospitals, warehouses, on the roads, and on sidewalks.
These deployed areas are both dynamic and frequently massive in scale.
The average size of a Walmart store is over 16,000 $m^{2}$ [@Walmart] and a single square block in Chicago is over 21,000 $m^{2}$ [CITATION NEEDED].
Retail and warehouse spaces can change drastically throughout the year and the state of roadways can be changing by the hour.
Much work has been made to address changing environments in robot perception [@stvl], but less has been built in open-source to represent maps of dynamic spaces.

For fully autonomous deployed systems to operate in these large and changing environments, they require tools that can be used to accurately map an area specified for their operation, update it over time, and scale to handle mapping of some of the largest indoor and outdoor spaces imaginable.
The field of Simultaneous Localization and Mapping (SLAM) aims to solve this problem using a variety of sensor modalities, including: laser scanners, radars, cameras, encoders, gps and IMUs.
The most commonly used perception sensor used for localization and mapping in industrial environments is the laser scanner. [@sensors]
SLAM methods using laser scanners are generally considered the most robust in the SLAM field and can provide accurate positioning in the presence of dynamic obstacles and changing environments. [@lidarslam]

Previously existing open-source laser scanner SLAM algorithms available to users in the popular Robot Operating System (ROS) include GMapping, Karto, Cartographer, and Hector.
However, few of these can build accurate maps of large spaces on the scale of the average Walmart store.
Even fewer can do so in real-time using the mobile processor typically found in mobile robot systems today.
The only package that could accomplish the above was Cartographer, however it was abandoned by Google and it is no longer maintained.

We propose a new fully open-source ROS package, SLAM Toolbox, to solve this problem.
SLAM Toolbox builds on the legacy of Open Karto, the open-source library from SRI International, providing not only accurate mapping algorithms, but a variety of other tools and improvements. [CITATION NEEDED]
SLAM Toolbox provides multiple modes of mapping depending on need, asynchronous and asynchronous, utilities such as kinematic map merging, a localization mode, multi-session mapping, new and improved graph optimization, substantially reduced compute time, and prototype lifelong and distributed mapping applications.

This package, `slam_toolbox` is open-source under an LGPLv2.1 at https://github.com/SteveMacenski/slam_toolbox.git and is available in every current ROS distribution.
It was also selected as the new default SLAM vendor in ROS 2, the second generation of robot operating systems, replacing GMapping.
SLAM Toolbox was integrated into the new ROS 2 Navigation2 project, providing real-time positioning in dynamic environments for autonomous navigation. [@macenski2020marathon2]
It has been shown to map spaces as large as 24,000 $m^{2}$, or 250,000 $ft^{2}$, in real-time by non-expert technicians.

# Related Work
SLAM algorithms can be classified into two groups: the earlier algorithms that use the Bayes-based filter approach[@thrun2005probabilistic], and newer GRAPH-based methods[@graphslam, @loopclosure]. Significant lidar-based implementation of 2D SLAM algorithms, available as ROS packages, are GMapping [@gmapping] and HectorSLAM [@hector] under the filter based category, while GRAPH-based implementations are provided by Cartographer [@cartographer] and KartoSLAM [@karto]. <br>
GMapping (2007) is one of the most commonly used SLAM libraries. It uses particle filter approach to SLAM for the purpose of building grid maps from 2D lidar data.<br>

HectorSLAM (2011) relies on lidar scan matching and 3D navigation filter based on EKF state estimation. This method focuses on real-time robot pose estimation and generates 2D map with high update rate. Unlike other mentioned methods, Hector does not use odometry data, which can cause inaccurate pose and map updates when lidar scans arrive at a lower rate, or when mapping large or featureless spaces.

KartoSLAM and Cartographer are both graph-based algorithms that store a graph of robot poses and features. Graph based algorithms have to maintain only the pose graph, which usually makes it more efficient in handling resources, especially while building maps of a large scale.<br>
KartoSLAM (2010) uses Sparse Pose Adjustment for handling both scan matching and loop-closure.<br>
Cartographer (2016) consists of front-end, which is in charge of scan matching and building trajectory and submaps, and back-end that does the loop closure procedure using SPA. Solver used for scan-matching in Cartographer is Ceres Solver[@ceres-solver]. Cartographer provides pure localization mode, when user has a satisfactory map for usage. It also provides data serialization for storing processed data.
Mapping with Cartographer requires tuning a large number of parameters for different types of terrains, which makes KartoSLAM more reliable when used in different environments. Taking out-of-the-box Cartographer parameters can lead to unsuccesful mapping, therefore it can make the library challenging to use. On the other hand, Cartographer has greater flexibility in configuring the mapping and can produce slightly better quality maps.

# Features
sync/async
large maps
utilities toolbox: kinematic map merging, pose-graph manipulation, rotate maps, continue sessions
localization embracing change: elastic pose-graph deformation. Fixed buffer of new scans, add new nodes/constraints, once expires free and remove from graph, reverts to original
plugin optimizaters
speed ups
ceres
online/offline
continous session mapping, complete data reconstruction and use, no artifacts. load last session raw, able to manipulate then or continue mapping and loop close between old data. 
the raw data is key for all the modes and tools no other slam provides.
prototype lifelong + distributed
>>100k qft
easy, nonexpert


# Configuration

To use `slam_toolbox`, there are are 53 potential parameters to tune.
Luckily, in the project, the authors provide several profiles for common tasks tuned for a typical mobile robot platform, but additional tuning for an application is necessary.
Many of the parameters are self explanatory, however in this section we will highlight a few needing additional clarification.

The parameter `ceres_loss_function` is used to determine the loss function, if any, to use when performing graph optimization on the pose-graph. For industrial-grade lidars on professional robots, this can remain the default of a Squared loss function. However, for cheaper hardware or working in environments with mirrors and glass, a Huber loss function may improve map quality. A Huber loss function on an industrial-grade robot may slightly lower the positioning quality.

There are three major parameters that impact the frequency with which the SLAM problem is updated. They are `minimum_travel_distance`, `minimum_travel_heading`, and `minimum_time_interval`. The minimum travel distance and heading are the values for the distance and turning angle changes required to trigger an update. Once either of these are met, an update will be attempted with the current laser scan. The minimum time interval parameter acts differently. It is the minimum time required between updates to be valid, but will not itself trigger an update. For an update to occur, both the minimum time interval must be met as well as one of the heading or distance changes. 

The `scan_buffer_size` is the local window of laser scans `slam_toolbox` will use to scan match against and build chains of scans. The shorter this is, the smaller number of local running scans there are. This is also used by the localization mode as the window of local scans to use. The longer this is in pure localization, the greater number of scans from the current session will be kept in the problem to potentially scan match against, in addition to the data from the prior session.

The `mode` parameter informs the graph optimizer plugin, Ceres, about the mode of operation it is working in. In `localization` mode, Ceres will cache an additional lookup table at the cost of roughly double the memory to speed up the removal of localization scans. In `mapping` mode, it will not but then may take longer to remove scans when in localization mode. This parameter has no meaning in SLAM, only in the pure localization executable.

For extremely large maps, the stack memory overhead of serializing and deserializing maps for multiple sessions can exceed the C++ standard default. The `stack_size_to_use` parameter allows users to define, in bytes, the size of the stack to use to allow for smooth serialization and deserialization. If users do not use multiple session features, they can ignore this parameter.

To utilize the different tools like map merging and pose-graph manipulation, the `enable_interactive_mode` must be set to true. This parameter enables the storage of additional information required for the interactive mode. However, this comes with non-trivial overhead so it should not be enabled when these features are not being used in a production environment.

# Robots Using `slam_toolbox`

SLAM Toolbox has been integrated, tested, and deployed on a number of robot platforms across the world by both industry and researchers.
It is also the default SLAM vendor for ROS 2.

A few known examples where SLAM Toolbox has been used or is being used are:

- Simbe Robotics' Tally
- ROBOTIS' Turtlebot3
- Samsung Research America and Russia's research teams
- Rover Robotics' Rover
- Pal Robotics ARI
- Intel's Open Source Group
- Queensland University of Technology researchers
- Robosynthesis' EXTRM SC2.0
- Unbounded Robotics UBR-1
- Byte Robotics' Platform
- Unmanned Life's Platform
- MT Robot AG
- Magazino robot datasets
- 6 River Systems

# Acknowledgements

We acknowledge this work was largely developed at Simbe Robotics and later continued open-source support and development at Samsung Research.

# References

# TODOs

Ex paper reference: https://joss.readthedocs.io/en/latest/submitting.html#example-paper-and-bibliography

- [Steve] Features section
- [Ivona] related works section
- [Ivona] citations
- [Ivona] spelling / grammar
- [Steve] figures from roscon presentation
- [Steve] submit
