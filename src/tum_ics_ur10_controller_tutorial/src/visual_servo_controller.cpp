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
      target_pose_received_(false),
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
      X_target_B_ << -0.636946, -0.163941, 0.716149; 
      
      // Subscribe to the topic published by your YOLO/TF node
      target_sub_ = nh_.subscribe("/target_pose", 1, 
                    &VisualServoController::targetCallback, this);

      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cartesian_target_pose", 1);

      ROS_WARN_STREAM("=== VisualServoController constructed. Subscribing /target_pose (PoseStamped) ===");

    }

    VisualServoController::~VisualServoController()
    {
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

    void VisualServoController::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    { 
        geometry_msgs::PoseStamped pose_in, pose_B;
        pose_in.header.frame_id = "camera_rgb_optical_frame";
        pose_in.header.stamp = ros::Time(0); 
        pose_in.pose = msg->pose;

        try {
            // Transform once to Base Frame B
            tf_buffer_.transform(pose_in, pose_B, "ursa_base_link");

            std::lock_guard<std::mutex> lock(target_mutex_);

            double alpha_pos = 0.2;
            double alpha_ori = 0.01;
            
            // --- 1. Filter Linear Position (Frame B) ---
            cc::LinearPosition pos_meas_B(pose_B.pose.position.x, pose_B.pose.position.y, pose_B.pose.position.z);
            if (!target_received_) {
                X_target_B_ = pos_meas_B;
                target_received_ = true;
            } else {
                X_target_B_ = alpha_pos * pos_meas_B + (1 - alpha_pos)* X_target_B_;
            }

            // --- 2. Filter & Transform Orientation (Frame 0) ---
            Eigen::Quaterniond q_meas_B(pose_B.pose.orientation.w, pose_B.pose.orientation.x, 
                                        pose_B.pose.orientation.y, pose_B.pose.orientation.z);
            q_meas_B.normalize();

            // Slerp for smoothing
            if (!target_pose_received_) {
                target_quat_B_ = q_meas_B;
                target_pose_received_ = true;
            } else {
                if (q_meas_B.dot(target_quat_B_) < 0.0) q_meas_B.coeffs() *= -1.0;
                target_quat_B_ = target_quat_B_.slerp(alpha_ori, q_meas_B);
            }

            // CALCULATE FRAME 0 TARGET HERE
            //Eigen::Matrix3d R_B_to_0 = model_.T_0_B().rotation().matrix().transpose();
            cc::HomogeneousTransformation T_0_B = model_.T_0_B();
            Eigen::Matrix3d R_0_to_B = T_0_B.orientation().matrix();
            Eigen::Matrix3d R_B_to_0 = R_0_to_B.transpose();
            
            //turn around the z-axis
            Eigen::Matrix3d R_local_rot;
            R_local_rot = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());

            //O_target_0_ = R_B_to_0 * target_quat_B_.toRotationMatrix();
            O_target_0_ = R_B_to_0 * (target_quat_B_.toRotationMatrix() * R_local_rot);

            // CALCULATE FRAME 0 POSITION HERE
            // (X_0 = R_0_B * X_B + P_0_B)
            X_target_0_ = model_.T_0_B() * X_target_B_;

            last_target_time_ = ros::Time::now();
            
            //publish a transform with the recieved pose
            last_target_time_ = ros::Time::now();
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "ursa_base_link";  
            transformStamped.child_frame_id = "target_object";    
            transformStamped.transform.translation.x = pose_B.pose.position.x;
            transformStamped.transform.translation.y = pose_B.pose.position.y;
            transformStamped.transform.translation.z = pose_B.pose.position.z;
            //transformStamped.transform.rotation = pose_B.pose.orientation;
            transformStamped.transform.rotation.x = target_quat_B_.x();
            transformStamped.transform.rotation.y = target_quat_B_.y();
            transformStamped.transform.rotation.z = target_quat_B_.z();
            transformStamped.transform.rotation.w = target_quat_B_.w();
            tf_broadcaster_.sendTransform(transformStamped);


            // 3. Publish Visualizations immediately in B
            //publishMarker(X_target_B_, 1.0, 0.0, 0.0, 1); // Desired Red
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM_THROTTLE(2.0, "TF2 Exception: " << ex.what());
        }
    }

    Vector6d VisualServoController::update(const RobotTime &time, const JointState &current)
    {
      // 1. Check Control Mode
      if (!is_cartesian_flag_) return updateJoint(time, current);

      if (!startFlag_) {
        qStart_ = current.q;
        startFlag_ = true;
      }

      // 2. Fetch Current Robot State (Frame 0)
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::LinearPosition X_curr_0 = T_tool_0.position();
      cc::Rotation3 R_curr_0 = T_tool_0.orientation();

      // 3. Thread-Safe Target Acquisition (Frame 0)
      bool is_fresh = false;

      /*
      We need to make sure at all times, that we aren't passing garbage data between 
      the methods. For this we need to make sure that we are using the .matrix() method
      on our objects. When passing cc::LinearPosition or cc::Rotation3 objects to other 
      cc:... objects. If we are passing Eigen:: objects to cc:: we do not need the .matrix()
      method call. This method call performs a deep copy of the data.
      */

      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        double dt = (ros::Time::now() - last_target_time_).toSec();
        is_fresh = (target_pose_received_ && dt < 0.5);

        if (is_fresh) {
          Xd_0 = X_target_0_.matrix(); // Position in 0 (calculated in callback)
          Rd_0 = O_target_0_;//

          holding_pos_latched_ = false;
          ROS_INFO_THROTTLE(2.0, "Tracking Active Target.");
        } else {
          // Data lost: Latch position if first time, otherwise use latched values
          if (!holding_pos_latched_) {
            X_hold_0_ = X_curr_0.matrix();
            holding_pos_latched_ = true;
            ROS_WARN("Visual Target Lost -> Latching current pose.");
          }
          ROS_INFO_THROTTLE(2.0,"Holding Target.");
          Xd_0 = X_hold_0_.matrix();
          Rd_0 = O_hold_0_.matrix();
        }
      }

      // 4. Calculate Cartesian Error (Frame 0)
      cc::CartesianVector Delta_X;
      cc::AngularPosition O_tool_0_ = R_curr_0.matrix();


      cc::AngularPosition Od = Rd_0.matrix();

      Delta_X.linear() = X_curr_0 - Xd_0;
      Delta_X.angular() = -cc::logErrorWorld(Od, O_tool_0_);

      this->publishTargetPose(Xd_0, Rd_0.matrix());

      // 5. Kinematics & Jacobians (Frame 0)
      cc::Jacobian J = model_.J_tool_0(current.q);
      cc::Jacobian Jp = model_.Jp_tool_0(current.q, current.qp);
      
      Vector6d Xp = J * current.qp;
      Vector6d Xdp = Vector6d::Zero(); // Target velocity (static bottle)
      
      // Reference velocity in Task Space
      Vector6d Xrp = Xdp - Kp_ * Delta_X;
      Vector6d Xrpp = - Kp_ * (Xp - Xdp); 

      // 6. Joint Space Reference Generation
      cc::Jacobian::Inverse J_pinv = J.dampedPinv(0.15, 1.0);
      Vector6d Qrp = J_pinv * Xrp; 

      if (!Qrp.allFinite()) {
          ROS_ERROR_THROTTLE(1.0, "Singularity Detected! Qrp contains NaN. Command zeroed.");
          Qrp.setZero();
      }

      // Velocity Saturation
      double max_vel = 0.5; // rad/s
      double max_val = Qrp.lpNorm<Eigen::Infinity>();
      if (max_val > max_vel) Qrp *= (max_vel / max_val);

      // Reference Acceleration
      Vector6d Qrpp = J_pinv * (Xrpp - Jp * Qrp);

      // 7. Slotine-Li Control Law
      JointState js_r = current; 
      js_r.qp = Qrp;
      js_r.qpp = Qrpp;

      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);
      Vector6d Sq = current.qp - Qrp;
      
      // Parameter Adaptation (gamma = 1.0, dt = 0.002)
      if (Sq.norm() > 1e-6) {
        theta_ -= 1.0 * Yr_.transpose() * Sq * 0.002;
      } 

      // 8. Visualizations (B-Frame used only for RViz)
      cc::LinearPosition X_tool_B = model_.T_tool_B(current.q).position();
      publishMarker(X_tool_B, 0.0, 1.0, 0.0, 2); // Tool (Green)
      // Xd_B should be calculated in callback and stored as member for visualization
      publishMarker(X_target_B_, 1.0, 0.0, 0.0, 1); // Target (Red)

      return -Kd_ * Sq + Yr_ * theta_;
    }

    Vector6d VisualServoController::updateJoint(const RobotTime &time, const JointState &current)
    {
      if (!startFlag_)
      {
        qStart_ = current.q;
        startFlag_ = true;
      }

      // 1. Generate Trajectory (Joint Space)
      // vQd: [0]=pos, [1]=vel, [2]=accel
      VVector6d vQd = getJointPVT5(qStart_, goal_, time.tD(), 7.0);

      DeltaQ_ = current.q - vQd[0];
      DeltaQp_ = current.qp - vQd[1];

      // Reference state for Slotine-Li
      JointState js_r = current;
      js_r.qp = vQd[1] - Kp_ * DeltaQ_;
      js_r.qpp = vQd[2] - Kp_ * DeltaQp_;

      // 2. Coordinate Math for Switching & Visualization
      // We check distance in B-frame because it's our "World" reference
      cc::HomogeneousTransformation T_tool_B = model_.T_tool_B(current.q);
      cc::LinearPosition X_tool_B = T_tool_B.position();

      cc::HomogeneousTransformation T_goal_B = model_.T_tool_B(goal_); 
      cc::LinearPosition X_joint_goal_B = T_goal_B.position();

      // Visualization
      publishMarker(X_joint_goal_B, 1.0, 0.0, 0.0, 1); // Goal (Red)
      publishMarker(X_tool_B, 0.0, 1.0, 0.0, 2);       // Tool (Green)

      cc::LinearPosition pos_err_B = X_tool_B - X_joint_goal_B;

      // 3. Control Law (Joint Space)
      Yr_ = model_.regressor(vQd[1], vQd[2], js_r.qp, js_r.qpp);
      Vector6d Sq = current.qp - js_r.qp;
      
      // Parameter adaptation
      if (Sq.norm() > 1e-6) {
        theta_ -= 1.0 * Yr_.transpose() * Sq * 0.002;
      } 

      Vector6d tau = -Kd_ * Sq + Yr_ * theta_;

      // 4. Switching Logic
      if (pos_err_B.norm() < 0.01) // 1cm threshold
      {
        ROS_WARN("Joint Target reached. Switching to Cartesian Visual Servoing!");
        
        // Reset flags for next loop
        startFlag_ = false;
        is_cartesian_flag_ = true;

        // Lock and Latch the current pose to prevent jumps 
        // before the first visual target is processed.
        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          
          // Latch orientation in Frame 0 (Internal control frame)
          cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
          O_hold_0_ = T_tool_0.orientation();

          // Latch position in Frame 0 (Internal control frame)
          X_hold_0_ = T_tool_0.position();

          this->publishTargetPose(X_hold_0_, O_hold_0_);

          cc::HomogeneousTransformation T_start_B = model_.T_tool_B(current.q);
          X_target_B_ = T_start_B.position();


          // State flags
          have_valid_hold_ = true;
          holding_ori_latched_ = false; 
          holding_pos_latched_ = true; 
        }
      }

      // 5. Data Logging
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();
      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = current.q(i);
        msg.qp[i] = current.qp(i);
        msg.torques[i] = tau(i);
        msg.qd[i] = vQd[0](i);
        msg.Dq[i] = DeltaQ_(i);
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

    void VisualServoController::publishTargetPose(const cc::LinearPosition& Xd_0, const cc::Rotation3& R_target_0)
    {
        // 1. Safety Check
        if (!Xd_0.allFinite() || !R_target_0.matrix().allFinite()){
          ROS_ERROR_STREAM("Rotation or Linear Position Not all Finite!");
          return;
        }

        // 2. Transform from Frame 0 to Frame B
        // X_B = T_B_0 * X_0
        cc::HomogeneousTransformation T_B_0 = model_.T_0_B();//.inverse();
        cc::LinearPosition X_B = T_B_0 * Xd_0;
        cc::Rotation3 R_B = T_B_0.rotation() * R_target_0;

        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "ursa_base_link"; // Correct Frame B ID

        msg.pose.position.x = X_B.x();
        msg.pose.position.y = X_B.y();
        msg.pose.position.z = X_B.z();

        // 3. Convert Matrix to Quaternion
        Eigen::Quaterniond q(R_B.matrix());
        
        if (std::abs(q.norm() - 1.0) > 0.1 || !std::isfinite(q.w())) {
            msg.pose.orientation.w = 1.0; 
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
        } else {
            q.normalize();
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();
        }

        target_pose_pub_.publish(msg);
    }

    void VisualServoController::printRotation(const std::string& label, const cc::Rotation3& R)
    {
        // 1. Convert ControlCore Rotation3 to Eigen Matrix
        Eigen::Matrix3d mat = R.matrix();
        
        // 2. Convert Matrix to Euler Angles (ZYX convention: Yaw, Pitch, Roll)
        // Returns a vector: [0]=Yaw, [1]=Pitch, [2]=Roll
        Eigen::Vector3d euler = mat.eulerAngles(2, 1, 0);

        // 3. Convert Radians to Degrees
        double yaw   = euler[0] * 57.2958;
        double pitch = euler[1] * 57.2958;
        double roll  = euler[2] * 57.2958;

        ROS_INFO_STREAM(label << " [RPY deg]: " 
            << std::fixed << std::setprecision(2)
            << "R: " << roll << ", P: " << pitch << ", Y: " << yaw);
    }

    bool VisualServoController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli