#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

#include <amr_msgs/MoveToAction.h>

class Bug2Node
{

public:

  /** Node constructor.
    *
    * Creates all required servers, publishers, and listeners. */
  Bug2Node()
  : state_(STATE_IDLE)
  , transform_listener_(ros::Duration(10))
  {
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    // Publishers
    current_goal_publisher_ = pn.advertise<geometry_msgs::PoseStamped>("current_goal", 0, true); // enable "latching" on a connection
    action_goal_publisher_ = pn.advertise<amr_msgs::MoveToActionGoal>("move_to/goal", 1);
    // Subscribers
    simple_goal_subscriber_ = pn.subscribe<geometry_msgs::PoseStamped>("move_to_simple/goal", 1, boost::bind(&Bug2Node::simpleGoalCallback, this, _1));
    // Action server
    move_to_server_ = MoveToActionServerUPtr(new MoveToActionServer(pn, "move_to", boost::bind(&Bug2Node::moveToCallback, this, _1), false));
    move_to_server_->start();
    // Action client
    move_to_client_ = MoveToActionClientUPtr(new MoveToActionClient("motion_controller/move_to", false));
    ROS_INFO("Started [bug2] node.");
  }

  /** This callback is triggered when someone sends an action command to the
    * "move_to" server. */
  void moveToCallback(const amr_msgs::MoveToGoalConstPtr& goal)
  {
    ROS_INFO("Received [move_to] action command.");

    if (!setNewGoal(goal))
      return;

    // Start an infinite loop where in each iteration we check the current
    // state of the execution and act accordingly. We also check if the goal
    // has been preempted (i.e. a new goal was given). The loop is terminated
    // if the goal was reached, or if the node itself shuts down.
    ros::Rate rate(50);
    ros::NodeHandle nh;
    while (nh.ok())
    {
      // Cancel ongoing activities and exit if the goal was aborted
      if (!move_to_server_->isActive())
      {
        stopExecutingCurrentAction();
        return;
      }

      // Process pending preemption requests
      if (move_to_server_->isPreemptRequested())
      {
        ROS_INFO("Action preemption requested.");

        // Stop the execution of the current action and notify the ActionServer
        // that we preempted.
        stopExecutingCurrentAction();
        move_to_server_->setPreempted();

        if (move_to_server_->isNewGoalAvailable())
          setNewGoal(move_to_server_->acceptNewGoal());
      }

      switch (state_)
      {
        case STATE_IDLE:
          return;
        case STATE_NEW_GOAL:
          move_to_client_->sendGoal(current_goal_);
          state_ = STATE_MOVING_TO_GOAL;
          break;
        case STATE_MOVING_TO_GOAL:
          switch (move_to_client_->getState().state_)
          {
            case actionlib::SimpleClientGoalState::PREEMPTED:
              stopExecutingCurrentAction();
              move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. Motion controller action was preempted.");
              state_ = STATE_IDLE;
              break;
            case actionlib::SimpleClientGoalState::ABORTED:
              ros::service::call("wallfollower/enable", empty_request_);
              state_ = STATE_WALLFOLLOWING;
              break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
              ROS_INFO("Goal was reached.");
              move_to_server_->setSucceeded(amr_msgs::MoveToResult(), "Goal reached.");
              state_ = STATE_IDLE;
            default:
              break;
          };
          break;
        case STATE_WALLFOLLOWING:
          // check if we are on the line
          break;
      }

      rate.sleep();
    }

    // We get here only if nh.ok() returned false, i.e. the node has received
    // a shutdown request
    move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. The node has been killed.");
  }

  /** This callback is triggered when someone sends a message with a new target
    * pose to the "move_to_simple/goal" topic.
    *
    * The function simply packs the supplied pose into an action message and
    * re-sends it to the action server for the execution. */
  void simpleGoalCallback(const geometry_msgs::PoseStampedConstPtr& target_pose)
  {
    ROS_INFO("Received target pose through the \"simple goal\" topic. Wrapping it in the action message and forwarding to the server.");
    amr_msgs::MoveToActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *target_pose;
    action_goal_publisher_.publish(action_goal);
  }

private:

      //ROS_INFO("The goal was reached.");

  /** Set new target pose as given in the goal message.
    *
    * Checks if the orientation provided in the target pose is valid.
    * Publishes the goal pose for the visualization purposes.
    *
    * @return true if the goal was accepted. */
  bool setNewGoal(const amr_msgs::MoveToGoalConstPtr& new_goal)
  {
    if (!isQuaternionValid(new_goal->target_pose.pose.orientation))
    {
      ROS_WARN("Aborted. Target pose has invalid quaternion.");
      move_to_server_->setAborted(amr_msgs::MoveToResult(), "Aborted. Target pose has invalid quaternion.");
      return false;
    }
    else
    {
      // Log the new target pose
      double x = new_goal->target_pose.pose.position.x;
      double y = new_goal->target_pose.pose.position.y;
      double yaw = tf::getYaw(new_goal->target_pose.pose.orientation);
      ROS_INFO("New target pose: [%.3f %.3f %.3f]", x, y, yaw);
      // Publish the new target pose for visualization purposes
      poseStampedMsgToTF(new_goal->target_pose, target_pose_);
      target_pose_.frame_id_ = "odom";
      current_goal_publisher_.publish(new_goal->target_pose);
      // Store the goal message so we can later pass it to motion_controller
      current_goal_ = *new_goal;
      state_ = STATE_NEW_GOAL;
      return true;
    }
  }

  /** Stop the execution of the current action.
    *
    * Depending on the state of the robot either disables the wallfollower node
    * or aborts the current goal on the motion_controller node. */
  void stopExecutingCurrentAction()
  {
    ROS_INFO("Stopping the execution of the current action...");
    switch (state_)
    {
      case STATE_IDLE:
      case STATE_NEW_GOAL:
        break;
      case STATE_MOVING_TO_GOAL:
        move_to_client_->cancelGoal();
        break;
      case STATE_WALLFOLLOWING:
        ros::service::call("wallfollower/disable", empty_request_);
        break;
    }
    state_ = STATE_IDLE;
  }

  /** Checks if the quaternion is a valid navigation goal, i.e. has non-zero
    * length and is close to vertical. */
  bool isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    // Ensure that the quaternion does not have NaN's or infinities
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      ROS_WARN("Quaternion has NaN's or infinities.");
      return false;
    }

    // Ensure that the length of the quaternion is not close to zero
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    if (tf_q.length2() < 1e-6)
    {
      ROS_WARN("Quaternion has length close to zero.");
      return false;
    }

    // Normalize the quaternion and check that it transforms the vertical
    // vector correctly
    tf_q.normalize();
    tf::Vector3 up(0, 0, 1);
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
    if (fabs(dot - 1) > 1e-3)
    {
      ROS_WARN("The z-axis of the quaternion is not close to vertical.");
      return false;
    }

    return true;
  }

  enum State
  {
    STATE_IDLE,
    STATE_NEW_GOAL,
    STATE_MOVING_TO_GOAL,
    STATE_WALLFOLLOWING,
  };

  State state_;

  typedef actionlib::SimpleActionServer<amr_msgs::MoveToAction> MoveToActionServer;
  typedef actionlib::SimpleActionClient<amr_msgs::MoveToAction> MoveToActionClient;
  typedef std::unique_ptr<MoveToActionServer> MoveToActionServerUPtr;
  typedef std::unique_ptr<MoveToActionClient> MoveToActionClientUPtr;

  /// Server to expose MoveTo action.
  MoveToActionServerUPtr move_to_server_;

  /// Client for MoveTo action interface exposed by the motion_controller node.
  MoveToActionClientUPtr move_to_client_;

  ros::Publisher velocity_publisher_;
  ros::Publisher current_goal_publisher_;
  ros::Publisher action_goal_publisher_;

  ros::Subscriber simple_goal_subscriber_;
  ros::Subscriber obstacles_subscriber_;

  tf::TransformListener transform_listener_;

  /// Current goal pose (in global reference frame).
  tf::Stamped<tf::Pose> target_pose_;

  /// Current goal message.
  amr_msgs::MoveToGoal current_goal_;

  /// Utility object to simplyfy calling enable/disable wallfollower service.
  std_srvs::Empty empty_request_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bug2");
  Bug2Node b2n;
  ros::spin();
  return 0;
}

