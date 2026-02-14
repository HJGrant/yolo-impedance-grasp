#ifndef UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H
#define UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ur_model/ur_model.h>
#include <control_core/trajectory/state_trajectories.h>
#include <control_core/types.h>
#include <control_core/trajectory/polynomial_trajectories.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
//#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h> 
#include <Eigen/Geometry>  

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class VisualServoController : public ControlEffort
    {
    private:
      ur::URModel model_; // Declare object "model"
      ur::URModel::Parameters theta_; // Declare parameter theta_ from class URModel
      ur::URModel::Regressor Yr_; // Declare robot regressor from URModel


      bool startFlag_;
      bool is_cartesian_flag_;
      bool is_circular_flag_;

      Vector6d qStart_;
      JointState qInit_;
      JointState qHome_;
      JointState qPark_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;

      Matrix6d Kp_;
      Matrix6d Kd_;
      Matrix6d Ki_;
      Vector6d goal_;
      double totalTime_;

      Vector6d DeltaQ_;
      Vector6d DeltaQp_;

      ros::Subscriber target_sub_;
      cc::LinearPosition X_target_pt_; // Latest point from ROS
      std::mutex target_mutex_;        // Thread safety
      void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      bool target_received_;
      // cc::Rotation3 O_target_;        // Desired orientation from target pose 
      Eigen::Quaterniond target_quat_B_; 
      bool target_pose_received_ = false;
      ros::Time last_target_time_;
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_{tf_buffer_};
      cc::LinearPosition X_hold_;
      bool holding_pos_latched_ = false;
      ros::Publisher marker_pub_;
      tf2_ros::TransformBroadcaster tf_broadcaster_;  

      double X_circ;
      double Y_circ;
      double PI;


    public:
      VisualServoController(double weight = 1.0, const QString &name = "SimpleEffortCtrl");

      ~VisualServoController();

      void setQInit(const JointState &qinit);

      void setQHome(const JointState &qhome);

      void setQPark(const JointState &qpark);

      void publishMarker(const cc::LinearPosition& pos, double r, double g, double b, int id);

      void visualizeTrajectory(const VVector6d& trajectory);

    private:
      bool init();

      bool start();

      /**
       * @brief update the controller
       * 
       * @param time holds the current robot time
       * @param current current jointstate q, qP of the real robot
       * @return Vector6d next torque command
       */
      Vector6d update(const RobotTime &time, const JointState &current);

      Vector6d updateCartesian(const RobotTime &time, const JointState &current);
      Vector6d updateJoint(const RobotTime &time, const JointState &current);

      bool stop();
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_CARTESIANSPACE_CONTROLLER_H