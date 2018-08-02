#!/bin/bash

# set simulator gravity
rosservice call /gazebo/set_physics_properties "
time_step: 0.001
max_update_rate: 1000.0
gravity:
  x: 0.0
  y: 0.0
  z: 0
ode_config:
  auto_disable_bodies: False
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"

# spawn template mesh model
rosrun gazebo_ros spawn_model -file `rospack find linemod_template_description`/urdf/linemod_template.urdf -urdf -x 0 -y 0 -z 1 -model linemod_template

# move camera
rosservice call /gazebo/set_model_state '{model_state: { model_name: camera_sim, pose: { position: { x: 0, y: 0.0 ,z: 2.7 }, orientation: {x: 0, y: 1, z: 0, w: 1} }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'

# move template model
rosservice call /gazebo/set_model_state '{model_state: { model_name: linemod_template, pose: { position: { x: 0.0, y: 0.0 ,z: 2.0 }, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 1.0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
