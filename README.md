# uw_dynamics
## Abstract:
Underwater biomimetic robots are biologically inspired robots that involve complex rigid body dynamics and fluid interactions which are relatively hard to compute, test
and validate compared to robots which use propellers/thrusters for their propulsion. This paper presents a scalable open-source simulation framework built upon gazebo and ROS for
underwater biomimetic robots that may or may not adhere to slender body approximation. The paper details the underwater rigid body dynamics, reactive and resistive forces acting on
the body, torque due to these forces and showcases simulations of various biomimetic robots that were simulated using the developed framework.

## Getting Started:
Clone the repository to the home directory,
```
$ git clone https://github.com/imsenthur/uw_dynamics.git
```
## Build:
```
$ cd uw_dynamics/build/
$ cmake ../
$ make
```
## Add it to your GAZEBO_PLUGIN_PATH:
```
$ export GAZEBO_PLUGIN_PATH=~/uw_dyn/build/devel/lib:$GAZEBO_PLUGIN_PATH
```
## Implementation:
```
$ <gazebo>
$   <plugin name="underwater_dynamics" filename="libuw_dynamics.so">
$     <fluid_density>997</fluid_density>
$   </plugin>
$ </gazebo>
```
*Add the plugin to your robot's URDF/SDF.*
![alt text](https://raw.githubusercontent.com/imsenthur/uw_dynamics/master/sim.png)

## Future work:
Seamless integration of UW_sim package with the developed framework to use UW_sim for rendering and gazebo for dynamics is currently under development.
http://wiki.ros.org/uwsim
