## Slam Toolbox

[![Get it from the Snap Store](https://snapcraft.io/static/images/badges/en/snap-store-black.svg)](https://snapcraft.io/slam-toolbox)

# Introduction

Slam Toolbox is a set of tools and capabilities for 2D planar SLAM built by [Steve Macenski](https://www.linkedin.com/in/steven-macenski-41a985101) while at [Simbe Robotics](simberobotics.com) and in my free time. 

This project contains the ability to do most everything any other available SLAM library, both free and paid, and more. This includes:
- Ordinary point-and-shoot 2D SLAM mobile robotics folks expect (start, map, save pgm file)
- life-long mapping (start, serialize, wait any time, restart anywhere, continue refining)
- an optimization-based localization mode (start, serialize, restart anywhere in Localization mode, optimization based localizer)
- synchronous and asynchronous modes
- kinematic map merging (with an elastic graph manipulation merging technique in the works)
- plugin-based optimization solvers with a new optimized Google Ceres based plugin
- RVIZ plugin for interating with the tools
- graph manipulation tools in RVIZ to manipulate nodes and connections during mapping
- Map serialization and lossless data storage
- ... more but those are the highlights 

For running on live production robots, I recommend using the snap: slam-toolbox, it has optimizations in it that make it about 10x faster. You need the deb/source install for the other developer level tools that don't need to be on the robot (rviz plugins, etc).

This package has been benchmarked mapping building at 5x+ realtime up to about 30,000 sqft and 3x realtime up to about 60,000 sqft. with the largest area (I'm aware of) used was a 145,000 sq.ft. building in sychronous mode (e.i. processing all scans, regardless of lag), and *much* larger spaces in asynchronous mode. 

<!-- image here large map-->

# LifeLong Mapping

<!-- Gif here-->

As it sounds, this will allow the user to update existing maps and serialize the data for use in other mapping sessions, something sorely lacking from most SLAM implementations and nearly all planar SLAM implementations. Other good libraries that do this include rtabmap and cartographer, though they themselves have their own quirks. This library provides the mechanics to save not only the data, but the pose graph, and associated metadata to work with. This has been used to create maps by merging techniques (taking 2 or more serialized objects and creating 1 globally consistent one) as well as continuous mapping techniques (updating 1, same, serialized map object over time and refining it). 

A user may build maps by optimizing/merging submap pose graphs (see next section), but if you'd like continue building a map object rather than merging a few together, we provide special tools for that. In the plugin interface you'll see a checkbox for Starting Near Dock, when selected, slam_toolbox will automatically scan match and correct the robot's starting position relative to the pose graph so you start cleanly with optimized relative odometry. This also allows you to not start at strictly the same point in space - just in the general region as your original map's starting point. This has been tested to up to 2-3 meters away from the starting point with relatively the same view of the environment with shockingly high recall. This is needed or the map may start to see double walls or otherwise pose graph not converging to the original form. 

Today we support two major modes:
- Starting from the first node (assuming to be a dock or dock-region)
- Starting from where you left off

In the very immediate future, there will be support for:
- Starting at any particular node - select a node ID to start near
- Starting in any particular area - indicate current pose in the map frame to start at (*cough* like from AMCL *cough*) 

References above suggest starting to map from your dock or charging areas, since those are unlikely to frequently change and its a globally known position in the map. Starting at the dock/charging/storage position or area provides a great deal of benefits. Rather than asking someone to start near an arbitrary set of coordinates -- or more correctly the first node in the pose graph, who's going to remember that? -- starting at the dock is an intuitive and simple way to make sure the robots are starting around where they should to continue updating an existing serialized map object. References to the above planned additions weaken the need for this since a user can start from where ever they like, but this is a good way to get started and easy to build an automated map updating pipeline around. None of this matters if you intend to do offline, or multi-robot mapping, which is discussed below. This application probably wants to merge existing maps and submaps into one larger map to share rather than live update maps. 

In order to do some operations quickly, I use NanoFlann to speed up some K-d tree searchs of the graph (shout out!).

# Localization

<!-- Gif here-->

Localization mode consists of 3 things:
- Loads existing serialized map into the node
- Maintains a rolling buffer of recent scans in the pose-graph
- After expiring from the buffer scans are removed and the underlying map is not affected

It is comparable to Cartographer's pure-localization mode. To enable, set `mode: localization` in the configuration file to allow for the Ceres plugin to set itself correctly to be able to quickly add *and remove* nodes and constraints from the pose graph. Also, on run, send the service request to Slam Toolbox to enter localization mode and the location to start at. The localization mode will automatically load you map, take the first scan and match it against the local area to further refine your estimated position, and start localizing. 

To minimize the amount of changes required for moving to this mode over AMCL, we also expose a subscriber to the `/initial_pose` topic used by AMCL to relocalize to a position, which also hooks up to the `2D Pose Estimation` tool in RVIZ. This way you can enter localization mode with our approach but continue to use the same API as you expect from AMCL for ease of integration.

In summary, this approach I dub `elastic pose-graph localization` is where we take existing map pose-graphs and localized with-in them with a rolling window of recent scans. This way we can localize in an existing map using the scan matcher, but not update the underlaying map long-term should something go wrong. It can be considered a replacement to AMCL and results is not needing any .pgm maps ever again. The lifelong mapping/continuous slam mode above will do better if you'd like to modify the underlying graph while moving.

Note: Be sure to **not** serialize the graph in localization mode, you will corrupt it!


## Tools 

### Plugin based Optimizers

I have created a pluginlib interface for the ScanSolver abstract class so that you can change optimizers on runtime to test many different ones if you like.

Then I generated plugins for a few different solvers that people might be interested in. I like to swap them out for benchmarking and make sure its the same code running for all. I have supported Ceres, G2O, SPA, and GTSAM. 

GTSAM/G2O/SPA is currently "unsupported" although all the code is there. They don't outperform Ceres settings I describe below so I stopped compiling them to save on build time, but they're there and work if you would like to use them. PRs to implement other optimizer plugins are welcome.

### Map Merging

#### Kinematic

<!-- Gif here -->

This uses RVIZ and the plugin to load any number of posegraphs that will show up in RVIZ under `map_N` and a set of interactive markers to allow you to move them around. Once you have them all positioned relative to each other in the way you like, you can merge the submaps into a global `map` which can be downloaded with your map server implementation of choice. 

More information in the RVIZ Plugin section below.

#### Pose Graph Merging

This is under development.

This is to solve the problem of merging many maps together with an initial guess of location in an elastic sense. This is something you just can't get if you don't have the full pose-graph and raw data to work with -- which we have from our continuous mapping work.

Hint: This is also really good for multi-robot map updating as well :)  

#### Optimization based

This uses RVIZ and the plugin to load any number of posegraphs that will show up in RVIZ under `map_N` and a set of interactive markers to allow you to move them around. Once you have them all positioned relative to each other in the way you like, it will use these relative transforms to offset the pose-graphs into a common frame and minimize the constraint error between them using the Ceres optimizer.  You can merge the submaps into a global `map` which can be downloaded with your map server implementation of choice. Think of this like populating N mappers into 1 global mapper.

Using just kinematic placement of the maps will give you some improvements over an image stiching/editing software since you have sub-pixel accuracy, but you're still a little screwed if your submaps aren't globally consistent and unwarped - this is an intermediate to help with that until the pose-graph merging tool is complete.

### RVIZ Plugin

An rviz plugin is furnished to help with manual loop closures and online / offline mapping. By default interactive mode is off (allowing you to move nodes) as this takes quite a toll on rviz. When you want to move nodes, tick the interactive box, move what you want, and save changes to prompt a manual loop closure. Clear if you made a mistake. When done, exit interactive mode again. 

There's also a tool to help you control online and offline data. You can at any time stop processing new scans or accepting new scans into the queue. This is desirable when you want to allow the package to catch up while the robot sits still (**This option is only meaningful in sychronous mode. In asynchronous mode the robot will never fall behind.**) or you want to stop processing new scans while you do a manual loop closure / manual "help". If there's more in the queue than you want, you may also clear it. 

Additionally there's exposed buttons for the serialization and deserialization services to load an old pose-graph to update and refine, or continue mapping, then save back to file. The "Start By Dock" checkbox will try to scan match against the first node (assuming you started at your dock) to give you an odometry estimate to start with. Another option is to start using an inputted position in the GUI or by calling the underlying service. Additionally, you can use the current odometric position estimation if you happened to have just paused the robot or not moved much between runs. Finally (and most usefully), you can use the RVIZ tool for **2D Pose Estimation** to tell it where to go in **localization mode** just like AMCL.

Additionally the RVIZ plugin will allow you to add serialized map files as submaps in RVIZ. They will be displayed with an interactive marker you can translate and rotate to match up, then generate a composite map with the Generate Map button. At that point the composite map is being broadcasted on the `/map` topic and you can save it with the `map_saver`.

It's recommended to always continue mapping near the dock, if that's not possible, look into the starting from pose or map merging techniques. This RVIZ plugin is mostly here as a debug utility, but if you often find yourself mapping areas using rviz already, I'd just have it open. All the RVIZ buttons are implemented using services that a master application can control. 

The interface is shown below. *It is a little outdated! Sorry, I need to update this.*

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

# Configuration

The following settings and options are exposed to you. My default configuration is given in `config` directory.

`solver_plugin` - The type of nonlinear solver to utilize for karto's scan solver. Options: `solver_plugins::CeresSolver`, `solver_plugins::SpaSolver`, `solver_plugins::G2oSolver`. Default: `solver_plugins::CeresSolver`.

`ceres_linear_solver` - The linear solver for Ceres to use. Options: `SPARSE_NORMAL_CHOLESKY`, `SPARSE_SCHUR`, `ITERATIVE_SCHUR`, `CGNR`. Defaults to `SPARSE_NORMAL_CHOLESKY`.

`ceres_preconditioner` - The preconditioner to use with that solver. Options: `JACOBI`, `IDENTITY` (none), `SCHUR_JACOBI`. Defaults to `JACOBI`.

`ceres_trust_strategy` - The trust region strategy. Line searach strategies are not exposed because they perform poorly for this use. Options: `LEVENBERG_MARQUARDT`, `DOGLEG`. Default: `LEVENBERG_MARQUARDT`.

`ceres_dogleg_type` - The dogleg strategy to use if the trust strategy is `DOGLEG`. Options: `TRADITIONAL_DOGLEG`, `SUBSPACE_DOGLEG`. Default: `TRADITIONAL_DOGLEG`

`ceres_loss_function` - The type of loss function to reject outlier measurements. None is equatable to a squared loss. Options: `None`, `HuberLoss`, `CauchyLoss`. Default: `None`.

---

`minimum_time_interval` - Minimum time between scans to add to scan queue. Default 0.5 seconds.

`map_update_interval` - Interval to update the map topic and pose graph visualizations. Default 10 seconds.

All other parameters, see SlamKarto documentation. They're all just the inputs to OpenKarto so that documentation would be identical as well. 

# Install

ROSDep will take care of the major things

```
rosdep install -q -y -r --from-paths src --ignore-src
```

I also have a Snap built for this that's super easy to install if you know snaps, named `slam-toolbox`.

```
sudo snap install slam-toolbox --beta --devmode
```

Run your catkin build procedure of choice.

You can run via `roslaunch slam_toolbox online_sync.launch`

# Etc

## Brief incursion into snaps

Snap are completely isolated containerized packages that one can run through the Canonical organization on a large number of Linux distributions. They're similar to Docker containers but it doesn't share the kernel or any of the libraries, and rather has everything internal as essentially a seperate partitioned operating system based on Ubuntu Core. 

We package up slam toolbox in this way for a nice multiple-on speed up in execution from a couple of pretty nuanced reasons in this particular project, but generally speaking you shouldn't expect a speedup from a snap. 

Since Snaps are totally isolated and there's no override flags like in Docker, there's only a couple of fixed directories that both the snap and the host system can write and read from, including SNAP_COMMON (usually in `/var/snap/[snap name]/common`). Therefore, this is the place that if you're serializing and deserializing maps, you need to have them accessible to that directory. 

You can optionally store all your serialized maps there, move maps there as needed, take maps from there after serialization, or do my favorite option and `link` the directories with `ln` to where ever you normally store your maps and you're wanting to dump your serialized map files. 

Example of `ln`:
```
#           Source                           Linked
sudo ln -s /home/steve/maps/serialized_map/ /var/snap/slam-toolbox/common
```

and then all you have to do when you specify a map to use is set the filename to `slam-toolbox/map_name` and it should work no matter if you're running in a snap, docker, or on bare metal. The `-s` makes a symbol link so rather than `/var/snap/slam-toolbox/common/*` containing the maps, `/var/snap/slam-toolbox/common/serialized_map/*` will. By default on bare metal, the maps will be saved in `.ros`

## Notes for documentation to fill in later

- Dock starting, mapping, continuing example
- Cloud mapping example
- Mapping from an estimated starting pose example (via amcl)
