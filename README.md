# mse_vt_eeros

Vertiefungsprojekt 1 an der OST zum MSE Medical Engineering

# Use it

Be sure you have ROS-2 Humble installed, Gazebo, RViz and Rqt.
Also ROS2-Humble has to be sourced in all the shells you use.

## Compile

Change into the main directory and use colcon to build and install:

```
$ colcon build --symlink-install
```

## Run

Run the EEROS-Application:

```
$ cd install
$ LD_LIBRARY_PATH="../../src/eeros-project/install-x86/lib/:$LD_LIBRARY_PATH" ./demo_package/bin/demo_package/demo_package
```

Change LD_LIBRARY_PATH to aherever you have installed EEROS.

Start the Package:

```
$ cd install
$ ros2 launch demo_package simulation.launch.py
```

