#TODOs
- submission: https://joss.theoj.org/papers/new
- citations
- figures / details from roscon presentation
- https://joss.readthedocs.io/en/latest/submitting.html#example-paper-and-bibliography

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
It has been shown to map spaces as large as 24,000 m^2, or 250,000 ft^2, in real-time by non-expert technicians.

# Related Work

# Features
sync/async
large maps
utilities toolbox
localization
plugin optimizaters
speed ups
ceres
online/offline
continous session mapping
prototype lifelong + distributed


# Robots Using `slam_toolbox`
tally
ros2 default
tb3
magazino
Cyberbotics
rover
Robosynthesis
6 river
unmanned life
many reserch groups



(also what reserach / softwre enabled by it)

# Acknowledgements

We acknowledge this work was largely developed at Simbe Robotics and later continued open-source support and development at Samsung Research.

# References
