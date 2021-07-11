# udacity-project5
### Udacity's nano degree program: Robotics Software Engineering  
**Project Title**: Home Service Robot   
**Project Goals**: 
- Program a home service robot that will autonomously map an environment and navigate to pick up and deliver objects.

### Requirements
#### Dependencies:
- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo])
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)

### Package Directory
``` 
├── README.md 
├── images
│   ├── ... .... 
├── CMakeLists.txt 
├── add_markers 
│   ├── launch 
│   │   └── home_service_rviz_config.launch 
│   └── src 
│       ├── add_markers.cpp 
│   ├──  ... ... 
├── config 
│   └── marker_config.yaml 
├── map 
│   ├── building 
│   │   ├── ... ... 
│   ├── myoffice.world 
│   ├── myoffice_map.pgm 
│   ├── myoffice_map.yaml 
├── pick_objects 
│   └── src 
│       ├── pick_objects.cpp 
│   ├──  ... ... 
├── rvizConfig 
│   └── home_service.rviz 
├── scripts 
│   ├── add_marker.sh 
│   ├── home_service.sh 
│   ├── pick_objects.sh 
│   ├── test_navigation.sh 
│   └── test_slam.sh 
├── slam_gmapping 
│   ├── gmapping 
│   |── ... ... 
├── turtlebot 
│   |── turtlebot_teleop 
│   |── ... ... 
├── turtlebot_interactions 
│   |── turtlebot_rviz_launchers 
│   |── ... ... 
|── turtlebot_simulator 
│   ├── turtlebot_gazebo 
│   |── ... ... 
```
### Tasks
1. Map environment (myoffice.world)
    - Import turtlebot into the environment. `turtlebot_world.launch`
    - Control the bot by using keyboard. `keyboard_teleop.launch`
    - Interface with SLAM package. `gmapping_demo.launch`
    - Visualize the map in rviz. `view_navigation.launch`
    - Replace turtlebot with `personalized robot`.
