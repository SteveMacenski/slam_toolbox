## Steve's Super Fun SLAM Time

Fork on slam_karto open via LGPLv3 with some secret sauce I'm working on.

### Solver Plugins

I have generated pluginlib plugins for a few different solvers that people might be interested in. I like to swap them out for benchmarking and make sure its the same code running for all. I have supported G2O, SPA, and GTSAM. I'm probably going to wrap my own Ceres one as well since no one's open-sourced it but I know _certain_ people are using it. 

GTSAM is currently "unsopported" although all the code is there. I was having infinite problems linking against the library. If someone wants to PR a fix, I can add support but I'm done trying.

All the plugins obviously have dependencies. GTSAM has that + MKL, SPA has ROS dependency of SBA, Ceres has ceres, and G2O has G2O. All aren't hard to install but if you only care about one, you might want to kill the compilation of the others so you can ignore installing them. 

### Refactor

I have refactored the code a bit to follow my stylings. I still have LGPL bindings from OpenKarto but I will plan to rewrite all of slam_karto to remove all bindings and licenses from it. For now its a fork with small modifications.

### ...