# Semantic map transform publisher

In order to be able to infer the position of the shops in relation to the robot from the semantic map, we need to be able to define a position of the shop which relates to the robot pose. This can be achieved using th the provided script.

## Running the script

```
rosrun semantic_map_transform_publisher transform_publisher.py
```

The node will try to access the mongodb to look for a saved transformation. If none can be found, the current position of the robot will be used as the origin of the semantic_map frame. To recalibrate the frame in case the odometry drifts or the starting position of the robot was not correct, you can use the service `/semantic_map_tf/calibrate` which is of type `std_srvs/Empty`.

Once started, the node is publishing a static tf transform that can be used to find the correct position of waypoints and shops to point at.