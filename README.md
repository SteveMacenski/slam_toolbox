## Slam Toolbox

For live running on robots, I recommend using the snap: slam-toolbox, it has optimizations in it that make it 10x faster. You need the deb/source install for the other developer level tools that don't need to be on the robot (rviz plugins, etc).

I've seen this way maps building at 5x+ realtime up to about 20,000 sqft and 3x realtime up to about 60,000 sqft. with the largest area (I'm aware of) used was a 145,000 sq.ft. building at this point. 

The core scan matcher is taken out of open_karto but alot of other heavy changes have been made in the processing of the scans in the lib sdk version in this repository. If you're familiar with Karto however it should still be approachable. 

## Introduction 

### Solver Plugins

I have generated pluginlib plugins for a few different solvers that people might be interested in. I like to swap them out for benchmarking and make sure its the same code running for all. I have supported Ceres, G2O, SPA, and GTSAM. I wrapped my own Ceres one as well since no one's open-sourced it but I know _certain_ people are using it. 

GTSAM is currently "unsupported" although all the code is there. I was having problems linking against the library. If someone wants to PR a fix, I can add support but I'm done trying. It definitely won't out perform the others.

I have spent a extremely long time working with Ceres to optimize it for creation of massive maps, so unless you feel your application is very unique, I'd use my recommended settings. 

I have commented out G2O to speed up build time as I really don't recommend using it. SPA + Ceres are the way to go here I believe. 

### Tools

Manual loop closure, pausing and resuming SLAM, interspection and modification of the pose graph, synchronous SLAM (no missing laser scans) for online or offline application alike without change, more exposed options, rviz plugin for visual interspection and assisting in mapping, complete serialization and deserialization of the entire data set to continue mapping or refine.

### Continuous SLAM

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


### Localization

Beta feature, still in development, but using `mode: localization` will right now use 2x the memory for the optimizer at the trade off of easier removal of parameter and residual blocks. This is in order to add and quickly remove contraints and nodes from the graph for a built-in localization module. Surrounds an approach I'll dub `elastic pose-graph localization` where we take existing map pose-graphs and localized with-in them with a rolling window of recent scans. This way we can localize in an existing map using this awesome scan matcher, but not long term update the map should something go wrong. It can be considered a replacement to AMCL and results is not needing any .pgm maps ever again. The continuous slam mode above will do better if you'd like to modify the underlying graph while moving.

TBA. 

### Map Merging 

## Kinematic

This uses RVIZ and the plugin to load any number of posegraphs that will show up in RVIZ under `map_N` and a set of interactive markers to allow you to move them around. Once you have them all positioned relative to each other in the way you like, you can merge the submaps into a global `map` which can be downloaded with your map server implementation of choice. 

## Pose Graph Merging

This is under development.

This is to solve the problem of merging many maps together with an initial guess of location in an elastic sense. This is something you just can't get if you don't have the full pose-graph and raw data to work with -- which we have from our continuous mapping work.

Hint: This is also really good for multi-robot map updating as well :)  

## Optimization based

This uses RVIZ and the plugin to load any number of posegraphs that will show up in RVIZ under `map_N` and a set of interactive markers to allow you to move them around. Once you have them all positioned relative to each other in the way you like, it will use these relative transforms to offset the pose-graphs into a common frame and minimize the constraint error between them using the Ceres optimizer.  You can merge the submaps into a global `map` which can be downloaded with your map server implementation of choice. Think of this like populating N mappers into 1 global mapper.

Using just kinematic placement of the maps will give you some improvements over an image stiching/editing software since you have sub-pixel accuracy, but you're still a little screwed if your submaps aren't globally consistent and unwarped - this is an intermediate to help with that until the pose-graph merging tool is complete.

Hint: This is also really good for multi-robot map updating as well :)  

### Rviz Plugin

An rviz plugin is furnished to help with manual loop closures and online / offline mapping. By default interactive mode is off (allowing you to move nodes) as this takes quite a toll on rviz. When you want to move nodes, tick the interactive box, move what you want, and save changes to prompt a manual loop closure. Clear if you made a mistake. When done, exit interactive mode again. 

There's also a tool to help you control online and offline data. You can at any time stop processing new scans or accepting new scans into the queue. this is desirable when you want to allow the package to catch up while the robot sits still or you want to stop processing new scans while you do a manual loop closure / manual "help". If there's more in the queue than you want, you may also clear it.

Additionally there's exposed buttons for the serialization and deserialization services to load an old pose-graph to update and refine, or continue mapping, then save back to file. The "Start By Dock" checkbox will try to scan match against the first node (at dock) to give you an odometry estimate to start with. Other option is to start using the last map->base_link transform available (probably from AMCL). In practice, you probaly dont have both of these running at once, so for an application you should get that value and feed it in to the service request for that mode, this application is primarily for demonstration. If using the current estimate checkbox, it will assume you're starting at the last position and may take a few nodes to converge together. It's recommended to always continue mapping near the dock, if that's not possible, look into the starting from pose or map merging techniques. Starting from current odom is not recommended unless you had to stop mapping and you're continuing to map a few minutes later with the *robot in the same spot*. It's mostly here as a debug utility, but if you often find yourself mapping areas in sprints and pausing between, this is valuable. 

The interface is shown below.

![rviz_plugin](/images/rviz_plugin.png?raw=true "Rviz Plugin")


### Metrics

If you're a weirdo like me and you want to see how I came up with the settings I had, see below.

![ceres_solver_comparison](https://user-images.githubusercontent.com/14944147/41576505-a6802d76-733c-11e8-8eca-334da2c8bd50.png)

The data sets present solve time vs number of nodes in the pose graph on a large dataset, as that is not open source, but suffice to say that the settings I recommend work well. I think anyone would be hardset in a normal application to exceed or find that another solver type is better (that super low curve on the bottom one, yeah, that's it). Benchmark on a low power 7th gen i7 machine.

### Performance

It can map _very_ large spaces with reasonable CPU and memory consumption. Default settings increase O(N) on number of elements in the pose graph. I recommend from extensive testing to use the SPARSE_NORMAL_CHOLESKY solver with Ceres and the SCHUR_JACOBI preconditioner. Using LM at the trust region strategy is comparable to the dogleg subspace strategy, but LM is much better supported so why argue with it. 

You can get away without a loss function if your odometry is good (ie likelihood for outliers is extremely low). If you have an abnormal application, I might recommend a HuberLoss function. All these options and more are available from the ROS parameter server.

## Parameters

On top of ordinary Slam Karto, the following settings and options are exposed to you. My default configuration is given in `config` directory.

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

## Installation

ROSDep will take care of the major things.

I also have a Snap built for this that's super easy to install if you know snaps, named `slam-toolbox`.

Run your catkin build procedure of choice.

You can run via `roslaunch slam_toolbox build_map_w_params.launch`

### Motivating Example / Tutorial

... Coming Soon to theatres near you ...


### Notes for documentation for myself to fill in later
- Dock starting, mapping, continuing example
- Cloud mapping example
- Mapping from an estimated starting pose example (via amcl)
- rviz plugin similification (setting dock,continue,estimated deserialization, pausing)
- explanation of the transform for the continue against node (map->base_link)
- explanation why the special case of node 0 relating to that (or does anyone care if it works?)


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
