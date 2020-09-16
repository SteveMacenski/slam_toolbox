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
The average size of a Walmart store is over 16,000 $m^{2}$ [@walmart] and a single square block in Chicago is over 21,000 $m^{2}$ [@chicago].
Retail and warehouse spaces can change drastically throughout the year and the state of roadways can be changing by the hour.
Much work has been made to address changing environments in robot perception [@stvl], but less has been built in open-source to represent maps of dynamic spaces.

For fully autonomous deployed systems to operate in these large and changing environments, they require tools that can be used to accurately map an area specified for their operation, update it over time, and scale to handle mapping of some of the largest indoor and outdoor spaces imaginable.
The field of Simultaneous Localization and Mapping (SLAM) aims to solve this problem using a variety of sensor modalities, including: laser scanners, radars, cameras, encoders, gps and IMUs.
The most commonly used perception sensor used for localization and mapping in industrial environments is the laser scanner [@sensors].
SLAM methods using laser scanners are generally considered the most robust in the SLAM field and can provide accurate positioning in the presence of dynamic obstacles and changing environments [@lidarslam].

Previously existing open-source laser scanner SLAM algorithms available to users in the popular Robot Operating System (ROS) include GMapping, Karto, Cartographer, and Hector.
However, few of these can build accurate maps of large spaces on the scale of the average Walmart store.
Even fewer can do so in real-time using the mobile processor typically found in mobile robot systems today.
The only package that could accomplish the above was Cartographer, however it was abandoned by Google and it is no longer maintained.

We propose a new fully open-source ROS package, SLAM Toolbox, to solve this problem.
SLAM Toolbox builds on the legacy of Open Karto [@karto], the open-source library from SRI International, providing not only accurate mapping algorithms, but a variety of other tools and improvements.
SLAM Toolbox provides multiple modes of mapping depending on need, asynchronous and asynchronous, utilities such as kinematic map merging, a localization mode, multi-session mapping, improved graph optimization, substantially reduced compute time, and prototype lifelong and distributed mapping applications.

This package, `slam_toolbox` is open-source under an LGPLv2.1 at https://github.com/SteveMacenski/slam_toolbox.git and is available in every current ROS distribution.
It was also selected as the new default SLAM vendor in ROS 2, the second generation of robot operating systems, replacing GMapping.
SLAM Toolbox was integrated into the new ROS 2 Navigation2 project, providing real-time positioning in dynamic environments for autonomous navigation [@macenski2020marathon2].
It has been shown to map spaces as large as 24,000 $m^{2}$, or 250,000 $ft^{2}$, in real-time by non-expert technicians.
An example map can be seen in \autoref{fig:store_map}.

![Retail store map created using SLAM Toolbox [@roscon]. \label{fig:store_map}](store_map.png)

# Related Work
SLAM algorithms can be classified into two groups: the earlier algorithms that use the Bayes-based filter approaches [@thrun2005probabilistic], and newer graph-based methods [@graphslam].
Significant filter-based implementations available as ROS packages are GMapping [@gmapping] and HectorSLAM [@hector].
Cartographer [@cartographer] and KartoSLAM [@karto] are the major graph-based implementations available.

GMapping is one of the most commonly used SLAM libraries, presented in 2007.
It uses particle filter approach to SLAM for the purpose of building grid maps from 2D lidar data.
However, GMapping is not well suited for large spaces and fails to accurately close loops at an industrial scale.
Additionally, filter-based approaches cannot be easily reinitialized with multiple sessions.

HectorSLAM relies on lidar scan matching and 3D navigation filter based on EKF state estimation.
This method focuses on real-time robot pose estimation and generates 2D map with high update rate.
Unlike other mentioned methods, Hector does not use odometry data, which can cause inaccurate pose and map updates when lidar scans arrive at a lower rate, or when mapping large or featureless spaces.
HectorSLAM however does not provide loop closure capabilities, making it unsuitable for reliable mapping of large spaces or when using laser scanners with low update rates.

KartoSLAM and Cartographer are both graph-based algorithms that store a graph of robot poses and features.
Graph-based algorithms have to maintain only the pose-graph, which usually makes it efficient in handling resources, especially while building maps of a large scale.
KartoSLAM uses Sparse Bundle Adjustment for loop-closure graph optimization.
Cartographer consists of front-end, which is in charge of scan matching and building trajectory and submaps, and back-end that does the loop closure procedure.
The graph solver used in Cartographer is Google Ceres [@ceres-solver].
Cartographer provides pure localization mode, when user has a satisfactory map for usage.
It also provides data serialization for storing processed submaps only.
However, Cartographer has stopped maintenance and support from Google and has been largely abandoned.
Further, it fails to build suitable maps for annotation and localization with other localization software packages on robotic platforms without exceptional odometry.
The software's unusual complexity makes it challenging to modify or resolve seemingly simple issues, making it not suitable for many applications.

# Features

SLAM Toolbox is able to map spaces effectively using mobile Intel CPUs commonly found on robots well in excess of 100,000 $ft^{2}$.
It can be done easily using untrained technicans typically hired to deploy robot solutions or remotely using monitoring systems. 
Some applications have been created to automatically map a space using SLAM Toolbox as well paired with exploration planners.

It can also serialize a current mapping session and deserialized at a later time to continue refining or expanding an existing map.
This serialization saves the complete raw data and pose-graph rather than submaps, as in Cartographer, allowing a variety of novel tools to be developed and more accurate multi-session mapping.
These utilities include manual pose-graph manipulation, whereas a user can manually manipulate the pose-graph nodes and data to rotate a map or assist in a challenging loop closure, shown in \autoref{fig:utils}.
It also includes kinematic map merging, the process of merging multiple serialized maps into a composite map.
A 3D visualizer plugin was also created to assist in utilization of these tools and the core SLAM library capabilities.
Many additional tools and utilities could be developed using this representation as well.

![Pose-graph manipulation in progress to manually match a node's laser scan to the map [@roscon]. \label{fig:utils}](utils.png)

It provides 3 major operation modes and executables: synchronous mapping, asynchronous mapping, and pure localization.
Synchronous mapping provides the ability to map and localize in a space keeping a buffer of measurements to add to the SLAM problem.
This can be adventitious when the quality of the map is of particular importance or when doing offline processing.
By contrast, the asynchronous mode will only process new measurements when the last measurement is completed and the new update criteria are met.
As a result, this will never lag behind real-time when running complex loop closures.
However, the map may not include all valid measurements if processing the last takes too long.
This mode is adventitious when the quality of real-time localization is of particular importance.
Both of these modes can be used for multi-session SLAM, the process of reloading a prior session and continuing to refine the pose-graph.
\autoref{fig:circuit} shows a map of a large office building created by partially mapping in one session and completing the map in another session.
This map has multiple loop closures between the two datasets and was later used with the pure-localization mode to navigate autonomously.

The pure localization mode cannot be used to persist changes in the environment.
Instead, it uses a rolling buffer of measurements in the current session and matches them against the original session(s) measurements and pose-graph.
The current session's measurements will be added to the pose-graph with new constraints and nodes in the graph.
This allows changes in the environment to be embraced to increase localization quality based on new features or moved objects.
Over time, the measurements in the rolling buffer will "expire" and be removed from the pose-graph and localization problem, reverting the pose-graph to its original state for that region.
The authors refer to this process as elastic pose-graph deformation.
An interesting side effect is that the pure-localization mode can be used for effective lidar odometry when paired with no prior mapping session data.
It will simply match against its local buffer and keep only a recent view of the environment, allowing lidar odometry to scale to infinite sized spaces.

![Large office building map created using multiple SLAM sessions [@roscon]. \label{fig:circuit}](circuit_launch.png)

Finally, many updates were made to the OpenKarto SLAM libary.
The measurement matching methods were restructured for a 10x speed-up enabling multi-threaded processing.
The provided Sparse Bundle Adjustment optimization interface was replaced with Google Ceres, providing faster and more flexible optimization settings.
Additionally, the optimizer interface was turned into a run-time dynamically loaded plugin interface to allow future developers to use the latest and greatest in optimization technologies without modifying the original code.
Serialization and deserializion support was enabled to allow for saving and reloading mapping sessions.
Finally, new processing modes and K-D tree search were developed to process measurements to enable localization and multi-session mapping.
Various smaller improvements and optimizations were also made but excluded for brevity.

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
