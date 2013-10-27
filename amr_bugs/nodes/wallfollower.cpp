#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <amr_msgs/Ranges.h>
#include <amr_srvs/SwitchRanger.h>

ros::Subscriber sonar_subscriber;
ros::Publisher velocities_publisher;

/** Sonar callback is triggered every time the Stage node publishes new data
  * to the sonar topic. */
void sonarCallback(const amr_msgs::Ranges::ConstPtr& msg)
{
  // TODO: implement wallfollowing logic
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wallfollwer");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the [/switch_ranger] service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the pioneer sonars are available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_pioneer";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled pioneer sonars.");
  }
  else
  {
    ROS_ERROR("Pioneer sonars are not available, shutting down.");
    return 1;
  }
  // Create subscriber and publisher.
  sonar_subscriber = nh.subscribe("/sonar_pioneer", 100, sonarCallback);
  velocities_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  // Start infinite loop.
  ROS_INFO("Started wallfollower node.");
  ros::spin();
  return 0;
}
