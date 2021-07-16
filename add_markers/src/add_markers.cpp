#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

uint8_t robot_reach_status = 0;

/* robot goal proximity callback function */
void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   robot_reach_status = msg->data;
   return;
}

/* main function */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_reached", 1, goalReachCallback);
  bool done = false;

  // Set the initial marker shape as a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  ROS_INFO("Subscribed to target zone");

  while (ros::ok()) {

    //Do this every cycle to ensure the subscriber receives the message
    ros::spinOnce();
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.
    marker.ns = "add_markers";
    marker.id = 0;
    // Set the marker type.
    marker.type = shape;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    // threshold = marker.scale.x;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    switch (robot_reach_status)
    {
      case 0: // publish pick-up marker
        {
          ROS_INFO("publishing pick-up marker");
          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;

	  marker.pose.position.x = -2.0;
	  marker.pose.position.y = 0.5;
	  marker.pose.orientation.w = 1.0;
          break;
        }

        case 1:   // robot reach pickup zone, delete pick-up marker
          {
            ROS_INFO("removing pick-up marker");
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          }

        case 2:   // robot reach drop-off zone
          {
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          }

        case 3:   // publish drop-off marker
          {
            ROS_INFO("adding drop-off marker");
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::Marker::ADD;

	    marker.pose.position.x = -3.0;
	    marker.pose.position.y = -3.5;
	    marker.pose.orientation.w = 1.5;
            done = true;
            break;
          }

    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    //Publish the marker
    marker_pub.publish(marker);

    // if last marker published and noted as done exit
    if (done) {
      ROS_INFO("Deivery Complete!");
      sleep(5);
      return 0;
      }

    r.sleep();
  }

  return 0;
}
