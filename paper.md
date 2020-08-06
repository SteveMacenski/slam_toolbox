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
    affiliation: "1, 2"
  - name: Ivona Jambrecic
    affiliation: "3"
affiliations:
 - name: Open-Source Robotics Engineering Lead, Samsung Research
   index: 1
 - name: Senior Software Engineer Robotics and Navigation Lead, Simbe Robotics
   index: 2
 - name: Software Engineering Intern, Simbe Robotics
   index: 3
date: 31 July 2020
bibliography: paper.bib

---

# Summary

Developments in the field of mobile robotics and autonomous driving have resulted in robots and vehicles in retail stores, hospitals, warehouses, on the roads, and on sidewalks.
These deployed areas are both dynamic and frequently massive in scale.
The average size of a Walmart store is over 16,000 m^2 [CITATION NEEDED] and a single square block in Chicago is over 21,000 m^2 [CITATION NEEDED].
Retail and warehouse spaces can change drastically throughout the year and the state of roadways can be changing by the hour.

For fully autonomous deployed systems to operate in these large and changing environments, they require tools that can be used to accurately map a given area it operates within, update it over time, and scale to handle some of the largest indoor and outdoor spaces imaginable.
The field of Simultaneous Localization and Mapping (SLAM) aims to solve this problem using a variety of sensor modalities.
The most common sensors used for localization and mapping in industrial environments is the laser scanner. [CITATION NEEDED]
SLAM methods using laser scanners are generally considered the most robust in the SLAM field and can provide accurate positioning in the presence of dynamic obstacles and changing environments. [CITATION NEEDED]

Prior existing open-source laser scanner SLAM algorithms available to users in the popular Robot Operating System (ROS) include GMapping, Karto, Cartographer, and Hector.
However, few of these can build accurate maps of large spaces on the scale of the average Walmart store.
Even fewer can do so in real-time using the mobile processor typically found on mobile robot systems today.
The only package that could accomplish the above was Cartographer, however it was abandoned by Google and it is no longer maintained.

We propose a new fully open-source ROS package, SLAM Toolbox, to solve this problem.
SLAM Toolbox builds on the legacy of Open Karto, the open-source library from SRI International, providing not only accurate mapping algorithms, but a variety of other tools and improvements. [CITATION NEEDED]
SLAM Toolbox provides multiple modes of mapping depending on need, asynchronous and asynchronous, utilities such as kinematic map merging, a localization mode, multi-session mapping, new and improved graph optimization, substantially reduced compute time, and prototype lifelong and distributed mapping applications.

This package, `slam_toolbox` is open-source under an LGPLv2.1 at https://github.com/SteveMacenski/slam_toolbox.git and is available in every current ROS distribution.
It was also selected as the new default SLAM vendor in ROS 2, the second generation of robot operating systems, replacing GMapping.
SLAM Toolbox was integrated into the new ROS 2 Navigation2 project, providing real-time positioning in dynamic environments for autonomous navigation. [CITATION NEEDED NAV2] 
It has been shown to map spaces as large as 24,000 m^2, or 250,000 ft^2, in real-time by non-expert technicians.

# Related Work

[CITATION NEEDED gmapping, karto, cartographer, hector]

# Features
sync/async
large maps
utilities toolbox
localization
pose graph manipulation
plugin optimizaters
speed ups
ceres
online/offline
continous session mapping
prototype lifelong + distributed

# Configuration

To use `slam_toolbox`, there are are 53 potential parameters to tune.
Luckily, in the project, the authors provide several profiles for common tasks tuned for a typical mobile robot platform.
Regardless, additional tuning for an application is necessary.
Many of the parameters are self explanatory, however in this section we will highlight a few needing additional clarification.

The parameter `ceres_loss_function` is used to determine the loss function, if any, to use when performing graph optimization on the pose-graph. For industrial-grade lidars on professional robots, this can remain the default of a Squared loss function. However, for cheaper hardware or working in environments with mirrors and glass, a Huber loss function may improve map quality. A Huber loss function on an industrial-grade robot may slightly lower the positioning quality.

There are three major parameters that impact the frequency the SLAM problem is updated. They are `minimum_travel_distance`, `minimum_travel_heading`, and `minimum_time_interval`. The minimum travel distance and heading are the values for the distance and turning angle changes required to trigger an update. Once either of these are met, an update will be attempted with the current laser scan. The minimum time interval parameter acts differently. It is the minimum time required between updates to be valid, but will not itself trigger an update. For update to occur, bot hthe minimum time interval must be met as well as one of the heading or distance changes. 

The `scan_buffer_size` is the local window of laser scans `slam_toolbox` will use to scan match against and build chains of scans. The shorter this is, the less local running scans there are. This is also used by the localization mode as the window of local scans to use. The longer this is in pure localization, the more scans from the current session will be kept in the problem to potentially scan match against in addition to the data from the prior session.

The `mode` parameter informs the graph optimizer plugin, Ceres, to the mode of operation it is working in. In `localization` mode, Ceres will cache an additional lookup table at the cost of roughly double the memory to speed up the removal of localization scans. In `mapping` mode, it will not but then may take longer to remove scans when in localization mode. This parameter has no meaning in SLAM, only in the pure localization executable.

For extremely large maps, the stack memory overhead of serializing and deserializing maps for multiple sessions can exceed the C++ standard default. The `stack_size_to_use` parameter allows users to define, in bytes, the size of the stack to use to allow for smooth serialization and deserialization. If users do not use multiple session features, they can ignore this parameter.

To utilize the different tools like map merging and pose-graph manipulation, the `enable_interactive_mode` must be set to true. This parameter enables the storage of additional information required for the interactive mode. However, this comes with non-trivial overhead so it should not be enabled when these features are not being used in a production environment.

# Robots Using `slam_toolbox`

SLAM Toolbox has been integrated, tested, and deployed on a number of robot platforms across the world by both industry and researchers.
It is also the default SLAM vendor for ROS 2.

A few known examples where SLAM Toolbox has been used or is being used are:

- Simbe Robotics' Tally
- ROBOTIS' Turtlebot3
- Samsung Research American anf Russia's research teams
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

#TODOs
- submission: https://joss.theoj.org/papers/new
- figures / details from roscon presentation
- https://joss.readthedocs.io/en/latest/submitting.html#example-paper-and-bibliography
- https://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf
- find way to cte STVL
