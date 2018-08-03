```
roslaunch camera_sim_gazebo camera_sim.launch
roscd linemod_3dpose_estimator
./set_simulator.sh
roslaunch linemod_3dpose_estimator linemod.launch
rosrun linemod_3dpose_estimator viewpoint_planner
```