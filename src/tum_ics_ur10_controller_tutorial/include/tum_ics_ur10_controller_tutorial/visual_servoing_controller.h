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

#include <atomic>
#include <thread>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class VisualServoController : public ControlEffort
    {
    private:

      enum class RoutineState {
        TRACK = 0,        // normal visual servo tracking
        PREGRASP,         // move to pre-grasp pose
        GRASP_POSE,       // move to final grasp pose
        GRASP_CLOSE,      // call gripper grasp service
        MOVE_TO_PLACE,    // move to place pose
        RELEASE_OPEN,     // call gripper release service
        DONE,
        FINISHED
      };

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

      std::mutex target_mutex_;        // Thread safety

      ros::Subscriber target_sub_;
      void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      cc::LinearPosition X_target_B_; // Latest point from ROS in B frame
      cc::LinearPosition X_target_0_; // Latest point from ROS in 0 frame
      cc::LinearPosition pos_meas_B; // Latest point from ROS in 0 frame
      Eigen::Quaterniond target_quat_B_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  //the recieved quaternion from the visual servoing
      Eigen::Quaterniond q_target_B_prev = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
      Eigen::Matrix3d O_target_0_; // the orientation target in the 0 frame
      cc::Rotation3 Rd_0;
      cc::Rotation3 Rd_0_rotated;
      cc::LinearPosition Xd_0;  // the desired linear ćoordinate

      // time to last target, and recieved flags
      ros::Time last_target_time_;
      bool target_received_ = false;
      bool target_pose_received_ = false;
      bool have_q_target_B_prev = false;

      // needed in the hold state
      cc::LinearPosition X_hold_0_;
      cc::Rotation3 O_hold_0_;
      bool holding_pos_latched_ = false;
      bool holding_ori_latched_ = false;
      bool have_valid_hold_ = false;

      // ---------------------------
      // Grasp routine configuration
      // ---------------------------
      RoutineState routine_state_{RoutineState::TRACK};

      // "Still" detection in B frame (world-like)
      cc::LinearPosition still_ref_pos_B_{cc::LinearPosition::Zero()};
      Eigen::Quaterniond still_ref_q_B_{Eigen::Quaterniond::Identity()};
      double still_accum_s_{0.0};
      double still_pos_eps_m_{0.008};           // 8mm
      double still_ang_eps_deg_{5.0};           // 5deg
      double still_required_s_{10.0};            // 3s
      bool still_ref_valid_{false};

      // Locked target (frame 0) for grasp routine
      cc::LinearPosition X_locked_0_{cc::LinearPosition::Zero()};
      cc::LinearPosition X_ee_locked_0_{cc::LinearPosition::Zero()};
      cc::Rotation3 R_locked_0_{cc::Rotation3::Identity()};
      cc::Rotation3 R_ee_locked_0_{cc::Rotation3::Identity()};
      bool locked_valid_{false};

      // Pregrasp / grasp offsets (in object frame; applied using locked rotation)
      double pregrasp_offset_m_{0.1};          // 10cm standoff
      double grasp_offset_m_{0.00};             // contact/center

      // Place pose (frame 0). You can override via params.
      cc::LinearPosition X_place_0_{cc::LinearPosition::Zero()};
      cc::Rotation3 R_place_0_{cc::Rotation3::Identity()};
      bool place_pose_set_{false};

      // Progress / thresholds
      double reach_pos_eps_m_{0.09};           // 6mm  0.006
      double reach_ang_eps_deg_{4.0};           // 4deg

      // Gripper service calls are blocking -> run in a worker thread
      std::atomic<bool> gripper_busy_{false};
      std::atomic<bool> gripper_done_{false};
      std::atomic<int>  gripper_ret_{0};
      std::thread gripper_thread_;

      // Wall time bookkeeping for dt
      ros::Time last_update_wall_{0.0};

      // Helper routines
      void resetStillDetectorLocked();
      bool updateStillDetector(double dt);
      bool isReached(const cc::LinearPosition& X_curr_0, const cc::Rotation3& R_curr_0,
                     const cc::LinearPosition& Xd_0, const cc::Rotation3& Rd_0) const;
      void startGripperCommand(const std::string& cmd);
      void setDefaultPlacePoseNeeded();
      void setDefaultPlacePoseIfNeeded();

      //visualization and transforms
      ros::Publisher marker_pub_;
      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_{tf_buffer_};
      tf2_ros::TransformBroadcaster tf_broadcaster_;

      ros::Publisher target_pose_pub_;
      void publishTargetPose(const cc::LinearPosition& Xd_0, const cc::Rotation3& R_target_0);
      void printRotation(const std::string& label, const cc::Rotation3& R);

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
