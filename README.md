## Slam Toolbox

| DockerHub  | [![Build Status](https://img.shields.io/docker/cloud/build/stevemacenski/slam-toolbox.svg?label=build)](https://hub.docker.com/r/stevemacenski/slam-toolbox) | [![Build Status](https://img.shields.io/docker/pulls/stevemacenski/slam-toolbox.svg?maxAge=2592000)](https://hub.docker.com/r/stevemacenski/slam-toolbox) |
|-----|----|----|
| **Build Farm** | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__slam_toolbox__ubuntu_bionic_amd64)](http://build.ros.org/view/Kbin_uX64/job/Mdev__slam_toolbox__ubuntu_bionic_amd64/) | N/A |

We've received feedback from users and have robots operating in the following environments with SLAM Toolbox:
- Retail
- Warehouses
- Libraries
- Research

### Cite This Work

You can find this work [here](https://joss.theoj.org/papers/10.21105/joss.02783) and clicking on the image below.

> Macenski, S., Jambrecic I., "SLAM Toolbox: SLAM for the dynamic world", Journal of Open Source Software, 6(61), 2783, 2021. 

> Macenski, S., "On Use of SLAM Toolbox, A fresh(er) look at mapping and localization for the dynamic world", ROSCon 2019.

[![IMAGE ALT TEXT](https://user-images.githubusercontent.com/14944147/74176653-f69beb80-4bec-11ea-906a-a233541a6064.png)](https://vimeo.com/378682207)

# Introduction


Slam Toolbox is a set of tools and capabilities for 2D SLAM built by [Steve Macenski](https://www.linkedin.com/in/steven-macenski-41a985101) while at [Simbe Robotics](https://www.simberobotics.com/) and in his free time. 

This project contains the ability to do most everything any other available SLAM library, both free and paid, and more. This includes:
- Ordinary point-and-shoot 2D SLAM mobile robotics folks expect (start, map, save pgm file) with some nice built in utilities like saving maps
- Continuing to refine, remap, or continue mapping a saved (serialized) pose-graph at any time
- life-long mapping: load a saved pose-graph continue mapping in a space while also removing extraneous information from newly added scans
- an optimization-based localization mode built on the pose-graph. Optionally run localization mode without a prior map for "lidar odometry" mode with local loop closures
- synchronous and asynchronous modes of mapping
- kinematic map merging (with an elastic graph manipulation merging technique in the works)
- plugin-based optimization solvers with a new optimized Google Ceres based plugin
- RVIZ plugin for interacting with the tools
- graph manipulation tools in RVIZ to manipulate nodes and connections during mapping
- Map serialization and lossless data storage
- ... more but those are the highlights 

For running on live production robots, I recommend using the snap or from the build farm: slam-toolbox, it has optimizations in it that make it about 10x faster. You need the deb/source install for the other developer level tools that don't need to be on the robot (rviz plugins, etc).

This package has been benchmarked mapping building at 5x+ realtime up to about 30,000 sqft and 3x realtime up to about 60,000 sqft. with the largest area (I'm aware of) used was a 200,000 sq.ft. building in synchronous mode (e.i. processing all scans, regardless of lag), and *much* larger spaces in asynchronous mode. 

The video below was collected at [Circuit Launch](https://www.circuitlaunch.com/) in Oakland, California. Thanks to [Silicon Valley Robotics](https://svrobo.org/) & Circuit Launch for being a testbed for some of this work. This data is currently available upon request, but its going to be included in a larger open-source dataset down the line. 

![map_image](/images/circuit_launch.gif?raw=true "Map Image")

An overview of how the map was generated is presented below:
![slam_toolbox_sync_diagram](/images/slam_toolbox_sync.png)
1. ROS Node: SLAM toolbox is run in synchronous mode, which generates a ROS node. This node subscribes to laser scan and odometry topics, and publishes map to odom transform and a map.
2. Get odometry and LIDAR data: A callback for the laser topic will generate a pose (using odometry) and a laser scan tied at that node. These PosedScan objects form a queue, which are processed by the algorithm.
3. Process Data: The queue of PosedScan objects are used to construct a pose graph; odometry is refined using laser scan matching. This pose graph is used to compute robot pose, and find loop closures. If a loop closure is found, the pose graph is optimized, and pose estimates are updated. Pose estimates are used to compute and publish a map to odom transform for the robot.
4. Mapping: Laser scans associated with each pose in the pose graph are used to construct and publish a map.

# 03/23/2021 Note On Serialized Files

As of 03/23/2021, the contents of the serialized files has changed. For all new users after this date, this regard this section it does not impact you.

If you have previously existing serialized files (e.g. not `pgm` maps, but `.posegraph` serialized slam sessions), after this date, you may need to take some action to maintain current features. Unfortunately, an ABI breaking change was required to be made in order to fix a very large bug affecting any 360 or non-axially-mounted LIDAR system.

[This Discourse post](https://discourse.ros.org/t/request-for-input-potential-existing-slam-toolbox-serialized-file-invalidation/19520) highlights the issues. The frame storing the scan data for the optimizer was incorrect leading to explosions or flipping of maps for 360 and non-axially-aligned robots when using conservative loss functions. This change permanently fixes this issue, however it changes the frame of reference that this data is stored and serialized in. If your system as a non-360 lidar and it is mounted with its frame aligned with the robot base frame, you're unlikely to notice a problem and can disregard this statement. For all others noticing issues, you have the following options:
- Use the `<distro>-devel-unfixed` branch rather than `<distro>-devel`, which contains the unfixed version of this distribution's release which will be maintained in parallel to the main branches to have an option to continue with your working solution
- Convert your serialized files into the new reference frame with an offline utility
- Take the raw data and rerun the SLAM sessions to get a new serialized file with the right content

More of the conversation can be seen on tickets #198 and #281. I apologize for the inconvenience, however this solves a very large bug that was impacting a large number of users. I've worked hard to make sure there's a viable path forward for everyone.

# LifeLong Mapping

<!--  Continuing mapping Gif here-->

LifeLong mapping is the concept of being able to map a space, completely or partially, and over time, refine and update that map as you continue to interact with the space. Our approach implements this and also takes care to allow for the application of operating in the cloud, as well as mapping with many robots in a shared space (cloud distributed mapping). While Slam Toolbox can also just be used for a point-and-shoot mapping of a space and saving that map as a .pgm file as maps are traditionally stored in, it also allows you to save the pose-graph and metadata losslessly to reload later with the same or different robot and continue to map the space. 

Our lifelong mapping consists of a few key steps
- Serialization and Deserialization to store and reload map information
- KD-Tree search matching to locate the robot in its position on reinitalization
- pose-graph optimizition based SLAM with 2D scan matching abstraction 

This will allow the user to create and update existing maps, then serialize the data for use in other mapping sessions, something sorely lacking from most SLAM implementations and nearly all planar SLAM implementations. Other good libraries that do this include RTab-Map and Cartoprapher, though they themselves have their own quirks that make them (in my opinion) unusable for production robotics applications. This library provides the mechanics to save not only the data, but the pose graph, and associated metadata to work with. This has been used to create maps by merging techniques (taking 2 or more serialized objects and creating 1 globally consistent one) as well as continuous mapping techniques (updating 1, same, serialized map object over time and refining it). The major benefit of this over RTab-Map or Cartoprapher is the maturity of the underlying (but heavily modified) `open_karto` library the project is based on. The scan matcher of Karto is well known as an extremely good matcher for 2D laser scans and modified versions of Karto can be found in companies across the world. 

Slam Toolbox supports all the major modes:
- Starting from a predefined dock (assuming to be near start region)
- Starting at any particular node - select a node ID to start near
- Starting in any particular area - indicate current pose in the map frame to start at, like AMCL 

In the RVIZ interface (see section below) you'll be able to re-localize in a map or continue mapping graphically or programatically using ROS services. 

On time of writing: there a **highly** experimental implementation of what I call "true lifelong" mapping that does support the method for removing nodes over time as well as adding nodes, this results in a true ability to map for life since the computation is bounded by removing extraneous or outdated information. Its recommended to run the non-full LifeLong mapping mode in the cloud for the increased computational burdens if you'd like to be continuously refining a map. However a real and desperately needed application of this is to have multi-session mapping to update just a section of the map or map half an area at a time to create a full (and then static) map for AMCL or Slam Toolbox localization mode, which this will handle in spades. The immediate plan is to create a mode within LifeLong mapping to decay old nodes to bound the computation and allow it to run on the edge by refining the experimental node. Continuing mapping (lifelong) should be used to build a complete map then switch to the pose-graph deformation localization mode until node decay is implemented, and **you should not see any substantial performance impacts**. 


# Localization

<!-- map refind local area localization Gif here-->

Localization mode consists of 3 things:
- Loads existing serialized map into the node
- Maintains a rolling buffer of recent scans in the pose-graph
- After expiring from the buffer scans are removed and the underlying map is not affected

Localization methods on image map files has been around for years and works relatively well. There has not been a great deal of work in academia to refine these algorithms to a degree that satesfies me. However SLAM is a rich and well benchmarked topic. The inspiration of this work was the concept of "Can we make localization, SLAM again?" such that we can take advantage of all the nice things about SLAM for localization, but remove the unbounded computational increase. 

To enable, set `mode: localization` in the configuration file to allow for the Ceres plugin to set itself correctly to be able to quickly add *and remove* nodes and constraints from the pose graph, but isn't strictly required, but a performance optimization. The localization mode will automatically load your pose graph, take the first scan and match it against the local area to further refine your estimated position, and start localizing. 

To minimize the amount of changes required for moving to this mode over AMCL, we also expose a subscriber to the `/initialpose` topic used by AMCL to relocalize to a position, which also hooks up to the `2D Pose Estimation` tool in RVIZ. This way you can enter localization mode with our approach but continue to use the same API as you expect from AMCL for ease of integration.

In summary, this approach I dub `elastic pose-graph localization` is where we take existing map pose-graphs and localized with-in them with a rolling window of recent scans. This way we can localize in an existing map using the scan matcher, but not update the underlaying map long-term should something go wrong. It can be considered a replacement to AMCL and results is not needing any .pgm maps ever again. The lifelong mapping/continuous slam mode above will do better if you'd like to modify the underlying graph while moving.

## Tools 

### Plugin based Optimizers

I have created a pluginlib interface for the ScanSolver abstract class so that you can change optimizers on runtime to test many different ones if you like.

Then I generated plugins for a few different solvers that people might be interested in. I like to swap them out for benchmarking and make sure its the same code running for all. I have supported Ceres, G2O, SPA, and GTSAM. 

GTSAM/G2O/SPA is currently "unsupported" although all the code is there. They don't outperform Ceres settings I describe below so I stopped compiling them to save on build time, but they're there and work if you would like to use them. PRs to implement other optimizer plugins are welcome.

### Map Merging - Example uses of serialized raw data & posegraphs

#### Kinematic

This uses RVIZ and the plugin to load any number of posegraphs that will show up in RVIZ under `map_N` and a set of interactive markers to allow you to move them around. Once you have them all positioned relative to each other in the way you like, you can merge the submaps into a global `map` which can be downloaded with your map server implementation of choice. 

It's more of a demonstration of other things you can do once you have the raw data to work with, but I don't suspect many people will get much use out of it unless you're used to stitching maps by hand.

More information in the RVIZ Plugin section below.

#### Pose Graph Merging

This is under development.

This is to solve the problem of merging many maps together with an initial guess of location in an elastic sense. This is something you just can't get if you don't have the full pose-graph and raw data to work with -- which we have from our continuous mapping work.

Hint: This is also really good for multi-robot map updating as well :)  

### RVIZ Plugin

An rviz plugin is furnished to help with manual loop closures and online / offline mapping. By default interactive mode is off (allowing you to move nodes) as this takes quite a toll on rviz. When you want to move nodes, tick the interactive box, move what you want, and save changes to prompt a manual loop closure. Clear if you made a mistake. When done, exit interactive mode again. 

There's also a tool to help you control online and offline data. You can at any time stop processing new scans or accepting new scans into the queue. This is desirable when you want to allow the package to catch up while the robot sits still (**This option is only meaningful in synchronous mode. In asynchronous mode the robot will never fall behind.**) or you want to stop processing new scans while you do a manual loop closure / manual "help". If there's more in the queue than you want, you may also clear it. 

Additionally there's exposed buttons for the serialization and deserialization services to load an old pose-graph to update and refine, or continue mapping, then save back to file. The "Start By Dock" checkbox will try to scan match against the first node (assuming you started at your dock) to give you an odometry estimate to start with. Another option is to start using an inputted position in the GUI or by calling the underlying service. Additionally, you can use the current odometric position estimation if you happened to have just paused the robot or not moved much between runs. Finally (and most usefully), you can use the RVIZ tool for **2D Pose Estimation** to tell it where to go in **localization mode** just like AMCL.

Additionally the RVIZ plugin will allow you to add serialized map files as submaps in RVIZ. They will be displayed with an interactive marker you can translate and rotate to match up, then generate a composite map with the Generate Map button. At that point the composite map is being broadcasted on the `/map` topic and you can save it with the `map_saver`.

It's recommended to always continue mapping near the dock, if that's not possible, look into the starting from pose or map merging techniques. This RVIZ plugin is mostly here as a debug utility, but if you often find yourself mapping areas using rviz already, I'd just have it open. All the RVIZ buttons are implemented using services that a master application can control. 

The interface is shown below.

![rviz_plugin](/images/rviz_plugin.png?raw=true "Rviz Plugin")

### Graph Manipulation

By enabling `Interactive Mode`, the graph nodes will change from markers to interactive markers which you can manipulate. When you move a node(s), you can Save Changes and it will send the updated position to the pose-graph and cause an optimization run to occur to change the pose-graph with your new node location. This is helpful if the robot gets pushed, slips, runs into a wall, or otherwise has drifting odometry and you would like to manually correct it. 

When a map is sufficiently large, the number of interactive markers in RVIZ may be too large and RVIZ may start to lag. I only recommend using this feature as a testing debug tool and not for production. However if you are able to make it work with 10,000 interactive markers, I'll merge that PR in a heartbeat. Otherwise I'd restrict the use of this feature to small maps or with limited time to make a quick change and return to static mode by unchecking the box.

## Metrics

If you're a weirdo like me and you want to see how I came up with the settings I had for the Ceres optimizer, see below.

![ceres_solver_comparison](https://user-images.githubusercontent.com/14944147/41576505-a6802d76-733c-11e8-8eca-334da2c8bd50.png)

The data sets present solve time vs number of nodes in the pose graph on a large dataset, as that is not open source, but suffice to say that the settings I recommend work well. I think anyone would be hardset in a normal application to exceed or find that another solver type is better (that super low curve on the bottom one, yeah, that's it). Benchmark on a low power 7th gen i7 machine.

It can map _very_ large spaces with reasonable CPU and memory consumption. My default settings increase O(N) on number of elements in the pose graph. I recommend from extensive testing to use the `SPARSE_NORMAL_CHOLESKY` solver with Ceres and the `SCHUR_JACOBI` preconditioner. Using `LM` at the trust region strategy is comparable to the dogleg subspace strategy, but `LM` is much better supported so why argue with it. 

You can get away without a loss function if your odometry is good (ie likelihood for outliers is extremely low). If you have an abnormal application or expect wheel slippage, I might recommend a `HuberLoss` function, which is a really good catch-all loss function if you're looking for a place to start. All these options and more are available from the ROS parameter server.

# API

The following are the services/topics that are exposed for use. See the rviz plugin for an implementation of their use. 

## Subscribed topics

| scan  | `sensor_msgs/LaserScan` | the input scan from your laser to utilize | 
|-----|----|----|
| **tf** | N/A | a valid transform from your configured odom_frame to base_frame |


## Published topics

| Topic  | Type | Description | 
|-----|----|----|
| map  | `nav_msgs/OccupancyGrid` | occupancy grid representation of the pose-graph at `map_update_interval` frequency | 
| pose | `geometry_msgs/PoseWithCovarianceStamped` | pose of the base_frame in the configured map_frame along with the covariance calculated from the scan match |

## Exposed Services

| Topic  | Type | Description | 
|-----|----|----|
| `/slam_toolbox/clear_changes`  | `slam_toolbox/Clear` | Clear all manual pose-graph manipulation changes pending | 
| `/slam_toolbox/deserialize_map`  | `slam_toolbox/DeserializePoseGraph` | Load a saved serialized pose-graph files from disk | 
| `/slam_toolbox/dynamic_map`  | `nav_msgs/OccupancyGrid` | Request the current state of the pose-graph as an occupancy grid | 
| `/slam_toolbox/manual_loop_closure`  | `slam_toolbox/LoopClosure` | Request the manual changes to the pose-graph pending to be processed | 
| `/slam_toolbox/pause_new_measurements`  | `slam_toolbox/Pause` | Pause processing of new incoming laser scans by the toolbox | 
| `/slam_toolbox/save_map`  | `slam_toolbox/SaveMap` | Save the map image file of the pose-graph that is useable for display or AMCL localization. It is a simple wrapper on `map_server/map_saver` but is useful. | 
| `/slam_toolbox/serialize_map`  | `slam_toolbox/SerializePoseGraph` | Save the map pose-graph and datathat is useable for continued mapping, slam_toolbox localization, offline manipulation, and more | 
| `/slam_toolbox/toggle_interactive_mode`  | `slam_toolbox/ToggleInteractive` | Toggling in and out of interactive mode, publishing interactive markers of the nodes and their positions to be updated in an application | 

# Configuration

The following settings and options are exposed to you. My default configuration is given in `config` directory.

## Solver Params

`solver_plugin` - The type of nonlinear solver to utilize for karto's scan solver. Options: `solver_plugins::CeresSolver`, `solver_plugins::SpaSolver`, `solver_plugins::G2oSolver`. Default: `solver_plugins::CeresSolver`.

`ceres_linear_solver` - The linear solver for Ceres to use. Options: `SPARSE_NORMAL_CHOLESKY`, `SPARSE_SCHUR`, `ITERATIVE_SCHUR`, `CGNR`. Defaults to `SPARSE_NORMAL_CHOLESKY`.

`ceres_preconditioner` - The preconditioner to use with that solver. Options: `JACOBI`, `IDENTITY` (none), `SCHUR_JACOBI`. Defaults to `JACOBI`.

`ceres_trust_strategy` - The trust region strategy. Line searach strategies are not exposed because they perform poorly for this use. Options: `LEVENBERG_MARQUARDT`, `DOGLEG`. Default: `LEVENBERG_MARQUARDT`.

`ceres_dogleg_type` - The dogleg strategy to use if the trust strategy is `DOGLEG`. Options: `TRADITIONAL_DOGLEG`, `SUBSPACE_DOGLEG`. Default: `TRADITIONAL_DOGLEG`

`ceres_loss_function` - The type of loss function to reject outlier measurements. None is equatable to a squared loss. Options: `None`, `HuberLoss`, `CauchyLoss`. Default: `None`.

`mode` - "mapping" or "localization" mode for performance optimizations in the Ceres problem creation

## Toolbox Params

`odom_frame` - Odometry frame

`map_frame` - Map frame

`base_frame` - Base frame

`map_file_name` - Name of the pose-graph file to load on startup if available

`map_start_pose` - Pose to start pose-graph mapping/localization in, if available

`map_start_at_dock` - Starting pose-graph loading at the dock (first node), if available. If both pose and dock are set, it will use pose

`debug_logging` - Change logger to debug

`throttle_scans` - Number of scans to throttle in synchronous mode

`transform_publish_period` - The map to odom transform publish period. 0 will not publish transforms

`map_update_interval` - Interval to update the 2D occupancy map for other applications / visualization

`enable_interactive_mode` - Whether or not to allow for interactive mode to be enabled. Interactive mode will retain a cache of laser scans mapped to their ID for visualization in interactive mode. As a result the memory for the process will increase. This is manually disabled in localization and lifelong modes since they would increase the memory utilization over time. Valid for either mapping or continued mapping modes.

`position_covariance_scale` - Amount to scale position covariance when publishing pose from scan match.  This can be used to tune the influence of the pose position in a downstream localization filter.  The covariance represents the uncertainty of the measurement, so scaling up the covariance will result in the pose position having less influence on downstream filters.  Default: 1.0

`yaw_covariance_scale` - Amount to scale yaw covariance when publishing pose from scan match.  See description of position_covariance_scale.  Default: 1.0

`resolution` - Resolution of the 2D occupancy map to generate

`max_laser_range` - Maximum laser range to use for 2D occupancy map rastering

`minimum_time_interval` - The minimum duration of time between scans to be processed in synchronous mode

`transform_timeout` - TF timeout for looking up transforms

`tf_buffer_duration` - Duration to store TF messages for lookup. Set high if running offline at multiple times speed in synchronous mode. 

`stack_size_to_use` - The number of bytes to reset the stack size to, to enable serialization/deserialization of files. A liberal default is 40000000, but less is fine.

`minimum_travel_distance` - Minimum distance of travel before processing a new scan

## Matcher Params

`use_scan_matching` - whether to use scan matching to refine odometric pose (uh, why would you not?)

`use_scan_barycenter` - Whether to use the barycenter or scan pose

`minimum_travel_heading` - Minimum changing in heading to justify an update

`scan_buffer_size` - The number of scans to buffer into a chain, also used as the number of scans in the circular buffer of localization mode

`scan_buffer_maximum_scan_distance` - Maximum distance of a scan from the pose before removing the scan from the buffer

`link_match_minimum_response_fine` - The threshold link matching algorithm response for fine resolution to pass 

`link_scan_maximum_distance` - Maximum distance between linked scans to be valid

`loop_search_maximum_distance` - Maximum threshold of distance for scans to be considered for loop closure 

`do_loop_closing` - Whether to do loop closure (if you're not sure, the answer is "true")

`loop_match_minimum_chain_size` - The minimum chain length of scans to look for loop closure

`loop_match_maximum_variance_coarse` - The threshold variance in coarse search to pass to refine

`loop_match_minimum_response_coarse` - The threshold response of the loop closure algorithm in coarse search to pass to refine

`loop_match_minimum_response_fine` - The threshold response of the loop closure algorithm in fine search to pass to refine

`correlation_search_space_dimension` - Search grid size to do scan correlation over

`correlation_search_space_resolution` - Search grid resolution to do scan correlation over

`correlation_search_space_smear_deviation` - Amount of multimodal smearing to smooth out responses

`loop_search_space_dimension` - Size of the search grid over the loop closure algorith

`loop_search_space_resolution` - Search grid resolution to do loop closure over

`loop_search_space_smear_deviation` - Amount of multimodal smearing to smooth out responses

`distance_variance_penalty` - A penalty to apply to a matched scan as it differs from the odometric pose

`angle_variance_penalty` - A penalty to apply to a matched scan as it differs from the odometric pose

`fine_search_angle_offset` - Range of angles to test for fine scan matching

`coarse_search_angle_offset` - Range of angles to test for coarse scan matching

`coarse_angle_resolution` - Resolution of angles over the Offset range to test in scan matching

`minimum_angle_penalty` - Smallest penalty an angle can have to ensure the size doesn't blow up

`minimum_distance_penalty` - Smallest penalty a scan can have to ensure the size doesn't blow up

`use_response_expansion` - Whether to automatically increase the search grid size if no viable match is found

# Install

ROSDep will take care of the major things

```
rosdep install -q -y -r --from-paths src --ignore-src
```

Also released in Melodic / Dashing to the ROS build farm to install debians.

Run your catkin build procedure of choice.

You can run via `roslaunch slam_toolbox online_sync.launch`

# Etc

## NanoFlann!

In order to do some operations quickly for continued mapping and localization, I make liberal use of NanoFlann (shout out!).


## Brief incursion into snaps

Snap are completely isolated containerized packages that one can run through the Canonical organization on a large number of Linux distributions. They're similar to Docker containers but it doesn't share the kernel or any of the libraries, and rather has everything internal as essentially a seperate partitioned operating system based on Ubuntu Core. 

We package up slam toolbox in this way for a nice multiple-on speed up in execution from a couple of pretty nuanced reasons in this particular project, but generally speaking you shouldn't expect a speedup from a snap. There's a generate snap script in the `snap` directory to create a snap.

Since Snaps are totally isolated and there's no override flags like in Docker, there's only a couple of fixed directories that both the snap and the host system can write and read from, including SNAP_COMMON (usually in `/var/snap/[snap name]/common`). Therefore, this is the place that if you're serializing and deserializing maps, you need to have them accessible to that directory. 

You can optionally store all your serialized maps there, move maps there as needed, take maps from there after serialization, or do my favorite option and `link` the directories with `ln` to where ever you normally store your maps and you're wanting to dump your serialized map files. 

Example of `ln`:
```
#           Source                           Linked
sudo ln -s /home/steve/maps/serialized_map/ /var/snap/slam-toolbox/common
```

and then all you have to do when you specify a map to use is set the filename to `slam-toolbox/map_name` and it should work no matter if you're running in a snap, docker, or on bare metal. The `-s` makes a symbol link so rather than `/var/snap/slam-toolbox/common/*` containing the maps, `/var/snap/slam-toolbox/common/serialized_map/*` will. By default on bare metal, the maps will be saved in `.ros`


## More Gifs!

![map_image](/images/mapping_steves_apartment.gif?raw=true "Map Image")

If someone from iRobot can use this to tell me my Roomba serial number by correlating to its maps, I'll buy them lunch and probably try to hire them.
