#include <tum_ics_ur10_controller_tutorial/visual_servoing_controller.h>

#include <tum_ics_ur_robot_msgs/ControlData.h>

#include <control_core/trajectory/state_trajectories.h>
#include <control_core/types.h>
#include <control_core/trajectory/polynomial_trajectories.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>




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
      is_circular_flag_(true)
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
      X_target_pt_ << 0.636946, 0.163941, 0.716149; 
      
      // Subscribe to the topic published by your YOLO/TF node
      target_sub_ = nh_.subscribe("/target_point", 1, 
                    &VisualServoController::targetCallback, this);

      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

    void VisualServoController::targetCallback(const geometry_msgs::Point::ConstPtr& msg)
    { 
        geometry_msgs::PointStamped point_in;
        geometry_msgs::PointStamped point_out;

        point_in.header.frame_id = "camera_rgb_optical_frame";
        point_in.header.stamp = ros::Time(0); // Get latest available transform
        point_in.point = *msg;

        try {
            // Transform the point from camera optical to robot base
            //tf_buffer_.transform(point_in, point_out, "ursa_base_link");
            tf_buffer_.transform(point_in, point_out, model_.get0Frame());

            std::lock_guard<std::mutex> lock(target_mutex_);
            target_received_ = true;
            last_target_time_ = ros::Time::now();

            // Apply alpha filter to the TRANSFORMED coordinates
            double alpha = 0.2; 
            X_target_pt_.x() = alpha * point_out.point.x + (1.0 - alpha) * X_target_pt_.x();
            X_target_pt_.y() = alpha * point_out.point.y + (1.0 - alpha) * X_target_pt_.y();
            X_target_pt_.z() = alpha * point_out.point.z + (1.0 - alpha) * X_target_pt_.z();

            ROS_INFO_STREAM_THROTTLE(1.0, "Target in Base Link: " << X_target_pt_.transpose());
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

      // 1. Get current Tool Pose (Frame 0)
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(current.q);
      cc::LinearPosition X_tool_0 = T_tool_0.position();
      cc::Rotation3 R_tool_0 = T_tool_0.orientation();

      // 2. Fetch Bottle Position (Ensure this is being sent in Frame 0!)
      cc::LinearPosition X_target_raw;
      bool is_data_fresh = false;
      {
          std::lock_guard<std::mutex> lock(target_mutex_);
          double dt_target = (ros::Time::now() - last_target_time_).toSec();
          is_data_fresh = target_received_ && (dt_target < 0.5);
          X_target_raw = X_target_pt_;
      }

      // 3. TARGET LOGIC
      cc::LinearPosition Xd;
      cc::AngularPosition Od = O_lock_; 

      if (!is_data_fresh) {
        if (!holding_pos_latched_) {
            X_hold_ = X_tool_0; // Capture only the first time data is lost
            holding_pos_latched_ = true;
        }
        Xd = X_hold_; 
      } else {
        holding_pos_latched_ = false; // Reset when tracking is active
        Xd = X_target_raw; 
      }

      publishMarker(Xd, 1.0, 0.0, 0.0, 0);
      publishMarker(X_tool_0, 0.0, 1.0, 0.0, 1);

      // 4. Calculate Cartesian Errors
      cc::CartesianVector Delta_X; 
      Delta_X.linear() << X_tool_0 - Xd;
      //Delta_X.angular() = -cc::logErrorWorld(Od, (cc::AngularPosition)R_tool_0);
      Delta_X.angular().setZero();

      ROS_WARN_STREAM_THROTTLE(1.0, "Linear Error Delta_X.linear: " << Delta_X.linear() << "\n");
      ROS_WARN_STREAM_THROTTLE(1.0, "Linear Error Delta_X.angular: " << Delta_X.angular() << "\n");

      // 5. Reference Velocity
      cc::Jacobian J_tool_0 = model_.J_tool_0(current.q);
      cc::Jacobian Jp_tool_0 = model_.Jp_tool_0(current.q, current.qp);
      
      Vector6d Xp = J_tool_0*current.qp;

      Vector6d Xdp = Vector6d::Zero();
      Vector6d Delta_Xp = Xp - Xdp;

      Vector6d Xrp = Xdp - Kp_ * Delta_X;
      Vector6d Xrpp = - Kp_*Delta_Xp; 

      cc::Jacobian::Inverse J_pinv = J_tool_0.dampedPinv(0.05, 0.5);
      Vector6d Qrp = J_pinv * Xrp; 
      Vector6d Qrpp = J_pinv*(Xrpp - Jp_tool_0*Qrp);

      // 6. Saturated Joint Space References
      JointState js_r;
      js_r = current; 
      js_r.qp = Qrp;
      js_r.qpp = Qrpp;

      //double cart_v_limit = 0.4; // Slightly faster for exact tracking
      //for(int i = 0; i < 6; i++) {
      //    js_r.qp(i) = std::max(std::min(Qrp_raw(i), cart_v_limit), -cart_v_limit);
      //}
      //js_r.qpp = Vector6d::Zero(); 

      // 7. Slotine-Li Control
      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);
      Vector6d Sx = Xp - Xrp;
      Vector6d Sq= J_pinv*Sx;

      cc::Scalar gamma_inv = 1.0;
      cc::Scalar dt = 0.002;
      

      if (Sq.norm() > 1e-6) {
        theta_ -= gamma_inv * Yr_.transpose() * Sq * dt;
      } 

      tau = -Kd_ * Sq + Yr_ * theta_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Cartesianspace Control \n");

      Vector6d tau_feedback = -Kd_ * Sq;
      Vector6d tau_feedforward = Yr_ * theta_;

      ROS_INFO_STREAM_THROTTLE(1.0, "Feedback Torque (Gains): " << tau_feedback.transpose());
      ROS_INFO_STREAM_THROTTLE(1.0, "Feedforward Torque (Model): " << tau_feedforward.transpose());

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

      DeltaQ_ = current.q - vQd[0];
      DeltaQp_ = current.qp - vQd[1];

      // 2. Define Reference State (with Saturation)
      JointState js_r = current;
      Vector6d qp_cmd = vQd[1] - Kp_ * DeltaQ_;

      double v_limit = 0.5; // Cap velocity at 0.5 rad/s
      for(int i=0; i<6; i++) {
          js_r.qp(i) = std::max(std::min(qp_cmd(i), v_limit), -v_limit);
      }

      // USE ONLY THIS for acceleration to stay safe
      js_r.qpp = vQd[2]; 

      // 3. Coordinate Math (Using Base Frame B to fix the 1.2m offset)
      cc::HomogeneousTransformation T_tool_B = model_.T_tool_B(current.q);
      cc::LinearPosition X_tool_0 = T_tool_B.position();

      cc::HomogeneousTransformation T_goal_B = model_.T_tool_B(goal_); 
      cc::LinearPosition X_joint_end = T_goal_B.position();

      publishMarker(X_joint_end, 1.0, 0.0, 0.0, 0);
      publishMarker(X_tool_0, 0.0, 1.0, 0.0, 1);

      cc::LinearPosition pos_err = X_tool_0 - X_joint_end;

      // 4. Slotine-Li Control Law
      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);
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

        cc::HomogeneousTransformation T_start = model_.T_tool_0(current.q);
        O_lock_ = T_start.orientation();
        X_hold_ = T_start.position(); 
        
        ROS_WARN_STREAM("Joint Target reached. Switching to Cartesian!");
        ROS_INFO_STREAM("Orientation O_lock_ in Base Link: " << O_lock_);
        ROS_INFO_STREAM("theta_ right before switching: " << theta_);
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

    bool VisualServoController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
