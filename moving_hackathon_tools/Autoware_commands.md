# Commands from autoware

All commands need:

```bash
source ~/Autoware/ros/install/local_setup.bash
```

To have the correct environment.


# Basic ones

These nodes are the basic ones to have the car localized.

## Velodyne transform (Baselink to Localizer)

```bash
# roslaunch runtime_manager setup_tf.launch x:=1.05 y:=0.0 z:=1.7 yaw:=0.0 pitch:=0.0 roll:=0.0 frame_id:=/base_link child_frame_id:=/velodyne period_in_ms:=10

rosrun tf static_transform_publisher 1.05 0.0 1.7 0.0 0.0 0.0 base_link velodyne 50
```

## Vehicle Model

For visualization purposes only.

```bash
roslaunch vehicle_description vehicle_model.launch
```

## Map pointcloud

```bash
rosrun map_file points_map_loader noupdate /home/sam/moving_2019_ws/src/2easy_team/pcd_maps/day2_map.pcd
```

## TF world->map

```bash
roslaunch $HOME/Autoware/ros/src/.config/tf/tf_local.launch
```

## Velodyne driver

```bash
roslaunch runtime_manager velodyne_vlp16.launch calibration:=
```


## voxel_grid_filter (pointcloud downsampler)

Needed for localization.

```bash
roslaunch points_downsampler points_downsample.launch node_name:=voxel_grid_filter points_topic:=/points_raw
```


## ndt_matching (localization)

Needs the map to be published. Needs the velodyne publishing at `/points_raw`.

```bash
roslaunch lidar_localizer ndt_matching.launch method_type:=0 use_odom:=False use_imu:=False imu_upside_down:=False imu_topic:=/imu_raw get_height:=False output_log_data:=False localizer=velodyne
```

# Extended

These nodes are all needed to move the car.

## vel_pose_connect

Publishes `/current_pose` and `/current_velocity` which other nodes need.

```bash
roslaunch autoware_connector vel_pose_connect.launch topic_pose_stamped:=/ndt_pose topic_twist_stamped:=/estimate_twist sim_mode:=False
```

## waypoint_loader

Loads the next set of waypoints to follow.

```bash
roslaunch waypoint_maker waypoint_loader.launch load_csv:=True multi_lane_csv:=/home/sam/moving_2019_ws/src/2easy_team/waypoints_data/mission_1_from_stop_to_after_curve.csv replanning_mode:=False realtime_tuning_mode:=False resample_mode:=True resample_interval:=1 replan_curve_mode:=False overwrite_vmax_mode:=False replan_endpoint_mode:=True velocity_max:=20 radius_thresh:=20 radius_min:=6 velocity_min:=4 accel_limit:=0.5 decel_limit:=0.3 velocity_offset:=4 braking_distance:=5 end_point_offset:=1
```

## path_select

Makes the next set of waypoints the objective to follow. Needs to be run after `waypoint_loader`.

```bash
rosrun lattice_planner path_select
```

## lattice_velocity_set

Needed to connect the actually make the waypoints to be executed (publishes in `/final_waypoints`).

```bash
roslaunch lattice_planner lattice_velocity_set.launch use_crosswalk_detection:=False
```

## twist_filter

Filters the speed (optionally) with lowpass gains.
Note that here we need to remap the input from `/twist_raw` to `/twist_raw_tmp` so
we can apply our own gains on *angular.z* for our car to turn properly.

```bash
roslaunch waypoint_follower twist_filter.launch
```

## pure_pursuit

Computes the commands to follow the waypoints.

```bash
roslaunch waypoint_follower pure_pursuit.launch is_linear_interpolation:=True publishes_for_steering_robot:=False param_flag_:=0
```


## lane_select

Needed to choose the set of waypoints to follow.

```bash
roslaunch lane_planner lane_select.launch
```


## lane_navi

```bash
roslaunch lane_planner lane_navi.launch velocity:=40 output_file:=/tmp/lane_waypoint.csv
```

## lane_rule

```bash
rosrun lane_planner lane_rule
```

## lane_stop

```bash
rosrun lane_planner lane_stop
```