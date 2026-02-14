#include <tum_ics_ur10_controller_tutorial/visual_servoing_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core/types.h>
#include <control_core/trajectory/polynomial_trajectories.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <mutex>


static inline void quatToRPY(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  // ZYX (yaw-pitch-roll)
  Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0,1,2);
  roll = rpy[0];
  pitch = rpy[1];
  yaw = rpy[2];
}

static inline std::string vec3ToStr(double x, double y, double z, int prec = 4)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed); oss<<std::setprecision(prec);
  oss << "[" << x << ", " << y << ", " << z << "]";
  return oss.str();
}

static inline std::string quatToStr(const Eigen::Quaterniond& q, int prec = 4)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed); oss<<std::setprecision(prec);
  oss << "[w:" << q.w() << ", x:" << q.x() << ", y:" << q.y() << ", z:" << q.z() << "]";
  return oss.str();
}


namespace {
  struct OriHoldState {
    cc::Rotation3 O_hold_0 = cc::Rotation3::Identity();  // desired orientation in Frame 0
    bool holding_ori_latched = false;                    // print latch warning once
    bool have_valid_hold = false;                        // whether O_hold_0 is initialized

    Eigen::Quaterniond q_target_B_prev = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    bool have_q_target_B_prev = false;
  };

  std::mutex g_ori_hold_mtx;
  std::unordered_map<const void*, OriHoldState> g_ori_hold;
}


namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    VisualServoController::VisualServoController(double weight, const QString &name) : 
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      startFlag_(false),
      Kp_(Matrix6d::Zero()),
      Kd_(Matrix6d::Zero()),
      Ki_(Matrix6d::Zero()),
      goal_(Vector6d::Zero()),
      totalTime_(10.0),
      DeltaQ_(Vector6d::Zero()),
      DeltaQp_(Vector6d::Zero()),
      model_("ur10_model"),
      is_cartesian_flag_(false),
      is_circular_flag_(true),
      target_received_(false),
      target_quat_B_(Eigen::Quaterniond::Identity())  
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("joint_space_control_data", 1);

      // load the initial parameters from the parameter server
      ROS_ERROR_STREAM("Initializing model!");
      if(!model_.initRequest(nh_))
      {
      ROS_ERROR_STREAM("Error: initializing model failed!");
      }

      // initialize the robots parameter vector
      theta_ = model_.parameterInitalGuess();

      // Default target (matches your previous X_start)
      // NOTE: This default was originally for Frame 0 (Z points up/down depending on yaml).
      // We will now transform targets to WORLD Frame (ursa_base_link), so we flip coordinates approx.
      X_target_pt_ << -0.636946, -0.163941, 0.716149; 
      
      // Subscribe to the topic published by your YOLO/TF node
      target_sub_ = nh_.subscribe("/target_pose", 1, 
                    &VisualServoController::targetCallback, this);

      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

      ROS_WARN_STREAM("=== VisualServoController constructed. Subscribing /target_pose (PoseStamped) ===");

      // generate circular trajectory
      if (is_circular_flag_)
      { 
        double PI = 3.14159265358979323846;
        double X_circ(0.0);
        double Y_circ(0.0);
        for (double t = 0.0; t <= 2*PI; t = t + 0.001)
        {

        X_circ = -0.9470 + 0.5*cos(t);
        Y_circ = -0.9470 + 0.5*sin(t);
        if (t >= 2*PI)
          t = 0.0;
        }
      } 

      // Init the per-instance hold state
      {
        std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
        g_ori_hold[this] = OriHoldState{};
      }
    }

    VisualServoController::~VisualServoController()
    {
      // cleanup the per-instance hold state
      {
        std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
        auto it = g_ori_hold.find(this);
        if (it != g_ori_hold.end()) g_ori_hold.erase(it);
      }
    }

    void VisualServoController::setQInit(const JointState &qinit)
    {
      qInit_ = qinit;
    }
    void VisualServoController::setQHome(const JointState &qhome)
    {
      qHome_ = qhome;
    }
    void VisualServoController::setQPark(const JointState &qpark)
    {
      qPark_ = qpark;
    }
    void VisualServoController::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    { 
        geometry_msgs::PoseStamped pose_in;
        geometry_msgs::PoseStamped pose_out;

        // 1. Pack the raw pose into a PoseStamped
        //    Assuming the camera publishes in "camera_rgb_optical_frame"
        pose_in.header.frame_id = "camera_rgb_optical_frame";
        pose_in.header.stamp = ros::Time(0); // Get latest available transform
        pose_in.pose = msg->pose;

        // -------- DEBUG: raw camera pose --------
        {
          Eigen::Quaterniond q_cam(msg->pose.orientation.w,
                                   msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z);
          if (q_cam.norm() > 1e-12) q_cam.normalize();
          double r,p,y; quatToRPY(q_cam, r,p,y);
          ROS_INFO_STREAM_THROTTLE(0.2,
            "[CB] Raw(Camera frame) Pos="
            << vec3ToStr(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)
            << "  Quat=" << quatToStr(q_cam)
            << "  RPY(rad)=" << vec3ToStr(r,p,y)
          );
        }

        try {
            // 2. Transform the pose from camera optical frame to the robot's base frame.
            //    "ursa_base_link" is the common base frame in your TF tree (Frame B).
            tf_buffer_.transform(pose_in, pose_out, "ursa_base_link");

            std::lock_guard<std::mutex> lock(target_mutex_);

            // Check if this is a "new" detection after a period of data loss
            double dt_since_last = (ros::Time::now() - last_target_time_).toSec();
            bool reset_filter = !target_received_ || (dt_since_last > 0.5);

            // 3. Smooth the TRANSFORMED coordinates (now in World/Base frame)
            //    We strictly filter the result of the transform, NOT the raw camera data.
            //    Using a "lag" filter (Alpha filter)
            double alpha = 0.2; 
            
            // On the FIRST valid reception (or re-acquisition), initialize the filter 
            // with the current measurement to avoid dragging from an old position.
            if (reset_filter) { 
               X_target_pt_.x() = pose_out.pose.position.x;
               X_target_pt_.y() = pose_out.pose.position.y;
               X_target_pt_.z() = pose_out.pose.position.z;
               target_received_ = true; // Mark as received
               ROS_WARN_STREAM("Target Re-acquired! Resetting filter.");
            } else {
               X_target_pt_.x() = alpha * pose_out.pose.position.x + (1.0 - alpha) * X_target_pt_.x();
               X_target_pt_.y() = alpha * pose_out.pose.position.y + (1.0 - alpha) * X_target_pt_.y();
               X_target_pt_.z() = alpha * pose_out.pose.position.z + (1.0 - alpha) * X_target_pt_.z();
            }

            // 4. Save target orientation in base frame (Frame B)
            Eigen::Quaterniond q_des(pose_out.pose.orientation.w,
                                     pose_out.pose.orientation.x,
                                     pose_out.pose.orientation.y,
                                     pose_out.pose.orientation.z);

            if (q_des.norm() > 1e-9) {
              q_des.normalize();

              // [ADDED] Quaternion hemisphere consistency to avoid sign flips (q and -q same)
              {
                std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
                auto &st = g_ori_hold[this];
                if (st.have_q_target_B_prev) {
                  if (q_des.dot(st.q_target_B_prev) < 0.0) {
                    q_des.coeffs() *= -1.0;
                  }
                }
                st.q_target_B_prev = q_des;
                st.have_q_target_B_prev = true;
              }

              target_quat_B_ = q_des;   
              {
                  ROS_INFO_STREAM("[CB] target_quat_B_ = " << quatToStr(target_quat_B_));
              }
              target_pose_received_ = true;
            }

            last_target_time_ = ros::Time::now();
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "ursa_base_link";  
            transformStamped.child_frame_id = "target_object";    
            transformStamped.transform.translation.x = pose_out.pose.position.x;
            transformStamped.transform.translation.y = pose_out.pose.position.y;
            transformStamped.transform.translation.z = pose_out.pose.position.z;
            transformStamped.transform.rotation = pose_out.pose.orientation;  
            tf_broadcaster_.sendTransform(transformStamped);

            {
              Eigen::Quaterniond q_base(pose_out.pose.orientation.w,
                                        pose_out.pose.orientation.x,
                                        pose_out.pose.orientation.y,
                                        pose_out.pose.orientation.z);
              if (q_base.norm() > 1e-12) q_base.normalize();
              double r,p,y; quatToRPY(q_base, r,p,y);

              ROS_INFO_STREAM_THROTTLE(0.2,
                "[CB] TF(Base frame=ursa_base_link) MeasPos="
                << vec3ToStr(pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z)
                << "  FiltPos=" << vec3ToStr(X_target_pt_.x(), X_target_pt_.y(), X_target_pt_.z())
                << "  Quat=" << quatToStr(q_base)
                << "  RPY(rad)=" << vec3ToStr(r,p,y)
              );
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM_THROTTLE(2.0, "TF2 Exception: " << ex.what());
        }
    }

    bool VisualServoController::init()
    {
      ROS_WARN_STREAM("VisualServoController::init: loading parameters");

      std::vector<double> vec;

      // check namespace
      std::string ns = "~visual_servo_controller";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("VisualServoController init(): Control gains not defined in:" << ns);
        m_error = true;
        return false;
      }

      // D GAINS
      ros::param::get(ns + "/gains_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_d: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (size_t i = 0; i < STD_DOF; i++)
      {
        Kd_(i, i) = vec[i];
      }
      ROS_WARN_STREAM("Kd: \n" << Kd_);

      // P GAINS
      ros::param::get(ns + "/gains_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        Kp_(i, i) = vec[i] / Kd_(i, i);
      }
      ROS_WARN_STREAM("Kp: \n" << Kp_);

      // GOAL
      ros::param::get(ns + "/goal", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_p: wrong number of dimensions:" << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++)
      {
        goal_(i) = vec[i];
      }
      
      // total time
      ros::param::get(ns + "/time", totalTime_);
      if (!(totalTime_ > 0))
      {
        ROS_ERROR_STREAM("totalTime_: is negative:" << totalTime_);
        totalTime_ = 100.0;
      }

      ROS_WARN_STREAM("Goal [DEG]: \n" << goal_.transpose());
      ROS_WARN_STREAM("Total Time [s]: " << totalTime_);
      goal_ = DEG2RAD(goal_);
      ROS_WARN_STREAM("Goal [RAD]: \n" << goal_.transpose());
      return true;
    }

    bool VisualServoController::start()
    {
      ROS_WARN_STREAM("VisualServoController::start");
      return true;
    }

    Vector6d VisualServoController::update(const RobotTime &time, const JointState &current)
    {
      if(!is_cartesian_flag_)
      {
        return updateJoint(time, current);
      }

      if (!startFlag_)
      {
        qStart_ = current.q;
        startFlag_ = true;
      }
      
      //model_.broadcastFrames(current.q, ros::Time::now()); 
      Vector6d tau = Vector6d::Zero();

      cc::HomogeneousTransformation T_tool_B = model_.T_tool_B(current.q);
      cc::LinearPosition X_tool_B = T_tool_B.position();

      // DEBUG: tool position in base frame
      ROS_INFO_STREAM_THROTTLE(0.2,
        "[UPD] Tool(Base B) Pos=" << vec3ToStr(X_tool_B.x(), X_tool_B.y(), X_tool_B.z())
        << "  t=" << time.tD()
      );

      // 2. Fetch Bottle Position (Ensure this is being sent in Frame B!)
      cc::LinearPosition X_target_raw;
      bool is_data_fresh = false;
      bool is_pose_fresh = false;
      cc::Rotation3 O_target_B_local = cc::Rotation3::Identity();
      double dt_target = 999.0;

      {
          std::lock_guard<std::mutex> lock(target_mutex_);
          dt_target = (ros::Time::now() - last_target_time_).toSec();
          is_data_fresh = target_received_ && (dt_target < 0.5);

          is_pose_fresh = target_pose_received_ && (dt_target < 0.5);

          X_target_raw = X_target_pt_;

          if (is_pose_fresh) {
              O_target_B_local = cc::Rotation3(target_quat_B_.toRotationMatrix());

              // Eigen::Matrix3d R_raw = target_quat_B_.toRotationMatrix();
              // Eigen::Matrix3d T_fixed;
              // T_fixed << 0, 0, 1,
              //       1, 0, 0,
              //       0, 1, 0;

              // Eigen::Matrix3d R_corrected = R_raw * T_fixed;
              
              // O_target_B_local = cc::Rotation3(R_corrected);
              
              // {
              //     Eigen::Quaterniond q_corrected(R_corrected);
              //     ROS_INFO_STREAM("[UPD] Corrected target quat = " << quatToStr(q_corrected));
              // }
              // {
              //     ROS_INFO_STREAM("[UPD] target_quat_B_ = " << quatToStr(target_quat_B_));
              // }
          }
      }

      ROS_INFO_STREAM_THROTTLE(0.5,
        "[UPD] Target freshness: dt=" << dt_target
        << "  pos_fresh=" << (is_data_fresh ? "YES" : "NO")
        << "  ori_fresh=" << (is_pose_fresh ? "YES" : "NO")
      );

      // 3. TARGET LOGIC
      cc::LinearPosition Xd;

      if (!is_data_fresh) {
        if (!holding_pos_latched_) {
            X_hold_ = X_tool_B; // Capture in Frame B
            holding_pos_latched_ = true;
            ROS_WARN_STREAM("Target lost -> latch hold position at current tool position (Base B).");
        }
        Xd = X_hold_;
        ROS_WARN_STREAM_THROTTLE(1.0, "[UPD] Holding Position Xd=" << vec3ToStr(Xd.x(), Xd.y(), Xd.z())); 
      } else {
        holding_pos_latched_ = false; // Reset when tracking is active
        Xd = X_target_raw; 
        ROS_WARN_STREAM_THROTTLE(1.0, "[UPD] Tracking Position Xd=" << vec3ToStr(Xd.x(), Xd.y(), Xd.z())); 
      }

      // Visualization: Direct in Frame B (ursa_base_link)
      publishMarker(Xd, 1.0, 0.0, 0.0, 1);
      publishMarker(X_tool_B, 0.0, 1.0, 0.0, 2);

      // Compute Control Error for the Linear Position
      cc::LinearPosition Delta_X_lin_B = X_tool_B - Xd;

      // DEBUG: position error in base
      ROS_INFO_STREAM_THROTTLE(0.2,
        "[UPD] PosErr(Base B) dX=" << vec3ToStr(Delta_X_lin_B.x(), Delta_X_lin_B.y(), Delta_X_lin_B.z())
        << "  |dX|=" << Delta_X_lin_B.norm()
      );

      // rotate the linear error in the B-frame into the 0-frame
      cc::LinearPosition Delta_X_lin_0 = model_.T_0_B().rotation() * Delta_X_lin_B;

      // DEBUG: position error in 0
      ROS_INFO_STREAM_THROTTLE(0.2,
        "[UPD] PosErr(Frame 0) dX0=" << vec3ToStr(Delta_X_lin_0.x(), Delta_X_lin_0.y(), Delta_X_lin_0.z())
        << "  |dX0|=" << Delta_X_lin_0.norm()
      );

      cc::CartesianVector Delta_X; 
      Delta_X.linear() = Delta_X_lin_0;

      // compute the error for the orientation
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::Rotation3 R_tool_0 = T_tool_0.orientation();
      cc::AngularPosition O_tool_0 = R_tool_0;

      // --- Ensure orientation hold is initialized when Cartesian control starts ---
      {
        std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
        auto &st = g_ori_hold[this];
        if (!st.have_valid_hold) {
          st.O_hold_0 = R_tool_0;         // initialize hold to current tool orientation
          st.have_valid_hold = true;
          st.holding_ori_latched = false;
          ROS_WARN_STREAM("Init O_hold_0 with current tool orientation (Frame 0).");
        }
      }

      {
        Eigen::Matrix3d R = R_tool_0;
        Eigen::Quaterniond q(R);
        if (q.norm() > 1e-12) q.normalize();
        double r,p,y; quatToRPY(q, r,p,y);
        ROS_INFO_STREAM_THROTTLE(0.5,
          "[UPD] Tool(Frame 0) OriQuat=" << quatToStr(q)
          << "  RPY(rad)=" << vec3ToStr(r,p,y)
        );
      }
      
      cc::AngularPosition Od;              
      bool using_target_ori = false;

      // Per-instance hold state (Frame 0)
      OriHoldState hold_state;
      {
        std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
        hold_state = g_ori_hold[this];
      }

      if (is_pose_fresh) {
        Eigen::Matrix3d R_0_B_eigen = model_.T_0_B().rotation().matrix();
        Eigen::Matrix3d R_target_B_eigen = target_quat_B_.toRotationMatrix();

        {
            Eigen::Quaterniond q_R0B(R_0_B_eigen);
            ROS_INFO_STREAM("[UPD] R_0_B_eigen quat = " << quatToStr(q_R0B));
            Eigen::Quaterniond q_target_B(R_target_B_eigen);
            ROS_INFO_STREAM("[UPD] R_target_B_eigen quat = " << quatToStr(q_target_B));
        }

        Eigen::Matrix3d R_target_0_eigen = R_0_B_eigen * R_target_B_eigen;
        Od = cc::AngularPosition(cc::Rotation3(R_target_0_eigen));
        using_target_ori = true;
        hold_state.holding_ori_latched = false;
        ROS_WARN_STREAM_THROTTLE(1.0, "[UPD] Tracking Orientation (Od = target orientation).");

        // Eigen::Matrix3d R_raw = target_quat_B_.toRotationMatrix();

        // Eigen::Matrix3d M_correct;
        // M_correct << 0, 0, 1,
        //             1, 0, 0,
        //             0, 1, 0;

        // Eigen::Matrix3d R_corrected = R_raw * M_correct;

        // {
        //     std::lock_guard<std::mutex> lock(target_mutex_);
        //     target_quat_B_ = Eigen::Quaterniond(R_corrected);
        // }

        // Eigen::Matrix3d R_0_B_eigen = model_.T_0_B().rotation().matrix();
        // Eigen::Matrix3d R_target_0_eigen = R_0_B_eigen * R_corrected;
        // Od = cc::AngularPosition(cc::Rotation3(R_target_0_eigen));

        // using_target_ori = true;
        // hold_state.holding_ori_latched = false;
        // ROS_WARN_STREAM_THROTTLE(1.0, "[UPD] Tracking Orientation (Od = target orientation after correction).");

     } else {
        if (!hold_state.have_valid_hold) {
          hold_state.O_hold_0 = R_tool_0;
          hold_state.have_valid_hold = true;
        }

        if (!hold_state.holding_ori_latched) {
          hold_state.O_hold_0 = R_tool_0;
          hold_state.have_valid_hold = true;
          hold_state.holding_ori_latched = true;
          ROS_WARN_STREAM("Target ori lost -> latch hold orientation at current tool orientation (Frame 0).");
        }

        Od = cc::AngularPosition(hold_state.O_hold_0);
        using_target_ori = false;

        ROS_WARN_STREAM_THROTTLE(1.0, "[UPD] Holding Orientation (Od = current tool orientation hold).");
      }

      // write back per-instance state
      {
        std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
        g_ori_hold[this] = hold_state;
      }

      // DEBUG: desired orientation in 0
      {
        Eigen::Matrix3d R = (cc::Rotation3)Od;
        Eigen::Quaterniond q(R);
        if (q.norm() > 1e-12) q.normalize();
        double r,p,y; quatToRPY(q, r,p,y);
        ROS_INFO_STREAM_THROTTLE(0.5,
          "[UPD] Desired(Frame 0) " << (using_target_ori ? "TARGET" : "HOLD_TOOL_AT_LOSS")
          << " OriQuat=" << quatToStr(q)
          << "  RPY(rad)=" << vec3ToStr(r,p,y)
        );
      }

      // Orientation error (log map)
      Delta_X.angular() = -cc::logErrorWorld(Od, O_tool_0);

      // DEBUG: orientation error magnitude (rad)
      ROS_INFO_STREAM_THROTTLE(0.2,
        "[UPD] OriErr(log) dO=" << vec3ToStr(Delta_X.angular()(0), Delta_X.angular()(1), Delta_X.angular()(2))
        << "  |dO|=" << Delta_X.angular().norm() << " rad"
      );

      // 5. Reference Velocity
      cc::Jacobian J_tool_0 = model_.J_tool_0(current.q);
      cc::Jacobian Jp_tool_0 = model_.Jp_tool_0(current.q, current.qp);
      
      Vector6d Xp = J_tool_0 * current.qp;

      Vector6d Xdp = Vector6d::Zero();
      Vector6d Delta_Xp = Xp - Xdp;

      Vector6d Xrp = Xdp - Kp_ * Delta_X;
      Vector6d Xrpp = - Kp_ * Delta_Xp; 

      cc::Jacobian::Inverse J_pinv = J_tool_0.dampedPinv(0.05, 0.5);
      Vector6d Qrp = J_pinv * Xrp; 

      // DEBUG: reference joint velocity
      ROS_INFO_STREAM_THROTTLE(0.5,
        "[UPD] Qrp (before sat) = " << Qrp.transpose()
      );
      
      // --- Velocity Saturation (Added) ---
      double max_vel = 0.5; // rad/s
      double scale = 1.0;
      double max_val = Qrp.lpNorm<Eigen::Infinity>(); // Max abs component
      if (max_val > max_vel) {
          scale = max_vel / max_val;
          Qrp = Qrp * scale;
      }

      ROS_INFO_STREAM_THROTTLE(0.5,
        "[UPD] Qrp (after sat) = " << Qrp.transpose()
        << "  scale=" << scale
      );

      Vector6d Qrpp;
      if (scale < 1.0) {
          // If saturated, assume constant velocity "cruise" -> zero acceleration
          Qrpp = Vector6d::Zero();
      } else {
          // Normal acceleration feedforward
          Qrpp = J_pinv * (Xrpp - Jp_tool_0 * Qrp);
      }
      // -----------------------------------

      // 6. Saturated Joint Space References
      JointState js_r;
      js_r = current; 
      js_r.qp = Qrp;
      js_r.qpp = Qrpp;

      // 7. Slotine-Li Control
      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);
      
      Vector6d Sq = current.qp - Qrp;

      cc::Scalar gamma_inv = 1.0;
      cc::Scalar dt = 0.002;
      
      if (Sq.norm() > 1e-6) {
        theta_ -= gamma_inv * Yr_.transpose() * Sq * dt;
      } 

      tau = -Kd_ * Sq + Yr_ * theta_;

      ROS_WARN_STREAM_THROTTLE(1.0,
        "[UPD] Cartesian Ctrl active. |Sq|=" << Sq.norm()
        << "  |tau|=" << tau.norm()
      );

      return tau;
    }

    Vector6d VisualServoController::updateJoint(const RobotTime &time, const JointState &current)
    {
      if (!startFlag_)
      {
        qStart_ = current.q;
        startFlag_ = true;
      }

      //model_.broadcastFrames(current.q, ros::Time::now()); 

      // 1. Generate Trajectory
      VVector6d vQd = getJointPVT5(qStart_, goal_, time.tD(), 7.0);
      //visualizeTrajectory(vQd);

      DeltaQ_ = current.q - vQd[0];
      DeltaQp_ = current.qp - vQd[1];

      JointState js_r;
      js_r = current;
      js_r.qp = vQd[1] - Kp_ * DeltaQ_;
      js_r.qpp = vQd[2] - Kp_ * DeltaQp_;


      // 3. Coordinate Math (Using Base Frame B to fix the 1.2m offset)
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::LinearPosition X_tool_0 = T_tool_0.position();

      cc::HomogeneousTransformation T_tool_B = model_.T_tool_B(current.q);
      cc::LinearPosition X_tool_B = T_tool_B.position();

      cc::HomogeneousTransformation T_goal_B = model_.T_tool_B(goal_); 
      cc::LinearPosition X_joint_end = T_goal_B.position();
      //cc::LinearPosition X_joint_end(0.8063, 0.1639, 0.6296);

      // Visualization: Transform from Frame 0 to Frame B (ursa_base_link) for consistency
      // But better yet, just trust T_tool_B if we want to be pure.
      cc::HomogeneousTransformation T_0_B = model_.T_0_B();
      publishMarker(X_joint_end, 1.0, 0.0, 0.0, 1); // Goal in B
      publishMarker(X_tool_B, 0.0, 1.0, 0.0, 2);    // Tool in B

      cc::LinearPosition pos_err = X_tool_B - X_joint_end;

      // DEBUG: joint mode status
      ROS_INFO_STREAM_THROTTLE(1.0,
        "[JNT] pos_err(norm)=" << pos_err.norm()
        << "  ToolB=" << vec3ToStr(X_tool_B.x(), X_tool_B.y(), X_tool_B.z())
        << "  GoalB=" << vec3ToStr(X_joint_end.x(), X_joint_end.y(), X_joint_end.z())
      );

      // 4. Slotine-Li Control Law
      Yr_ = model_.regressor(vQd[1], vQd[2], js_r.qp, js_r.qpp);
      Vector6d Sq = current.qp - js_r.qp;
      
      cc::Scalar gamma_inv = 1.0;
      cc::Scalar dt = 0.002;

      if (Sq.norm() > 1e-6) {
        theta_ -= gamma_inv * Yr_.transpose() * Sq * dt;
      } 

      Vector6d tau = -Kd_ * Sq + Yr_ * theta_;

      // 5. Switching Logic (Loosened threshold to 1cm)
      if (pos_err.norm() < 0.01) 
      {
        startFlag_ = false;
        is_cartesian_flag_ = true;

        cc::HomogeneousTransformation T_start_B = model_.T_tool_B(current.q);
        X_hold_ = T_start_B.position(); 

        cc::HomogeneousTransformation T_start_0 = model_.T_tool_0(current.q);
        // O_lock_ = T_start_0.orientation();
        
        ROS_WARN_STREAM("Joint Target reached. Switching to Cartesian!");
        //ROS_INFO_STREAM("Orientation O_lock_ in Base Link: " << O_lock_);
        //ROS_INFO_STREAM("theta_ right before switching: " << theta_);

        // Also initialize the orientation-hold state with current tool orientation,
        // so holding behavior is well-defined before first target pose arrives.
        {
          std::lock_guard<std::mutex> lk(g_ori_hold_mtx);
          auto &st = g_ori_hold[this];
          st.O_hold_0 = T_start_0.orientation();
          st.have_valid_hold = true;
          st.holding_ori_latched = false;
        }
      }
  
      // publish the ControlData (only for debugging)
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = current.q(i);
        msg.qp[i] = current.qp(i);
        msg.qpp[i] = current.qpp(i);

        msg.qd[i] = vQd[0](i);
        msg.qpd[i] = vQd[1](i);

        msg.Dq[i] = DeltaQ_(i);
        msg.Dqp[i] = DeltaQp_(i);

        msg.torques[i] = current.tau(i);
      }
      control_data_pub_.publish(msg);

      return tau;
      
    }

    void VisualServoController::publishMarker(const cc::LinearPosition& pos, double r, double g, double b, int id) {
        visualization_msgs::Marker marker;
        // VERY IMPORTANT: Set this to "ursa_base_link" or "world" 
        // depending on which frame your LinearPosition is calculated in!
        marker.header.frame_id = "ursa_base_link"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "target_position";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the position from your LinearPosition variable
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        
        marker.pose.orientation.w = 1.0; // Neutral orientation

        // Set the scale (size of the sphere)
        marker.scale.x = 0.05; // 5cm
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        // Set the color
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0; // Must be 1.0 (opaque) to see it!

        marker.lifetime = ros::Duration(); // Permanent until replaced
        marker_pub_.publish(marker);
    }

    void VisualServoController::visualizeTrajectory(const VVector6d& trajectory) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "ursa_base_link"; // Use the correct ROS frame
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "trajectory";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 2;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // Line width
        line_strip.scale.x = 0.01; 

        // Color: Blue for the planned path
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        for (const auto& q_step : trajectory) {
            // Use T_tool_B for correct World Frame coordinates!
            cc::HomogeneousTransformation T_tool_B = model_.T_tool_B(q_step);
            
            // Extract the position vector from the transformation
            cc::LinearPosition pos = T_tool_B.position();

            geometry_msgs::Point p;
            p.x = pos.x(); 
            p.y = pos.y();
            p.z = pos.z();
            line_strip.points.push_back(p);
        }

        marker_pub_.publish(line_strip);
    }

    bool VisualServoController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli