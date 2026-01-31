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
      is_circular_flag_(true),
      target_received_(false) 
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

        // 1. Pack the raw point into a PointStamped
        //    Assuming the camera publishes in "camera_rgb_optical_frame"
        point_in.header.frame_id = "camera_rgb_optical_frame";
        point_in.header.stamp = ros::Time(0); // Get latest available transform
        point_in.point = *msg;

        try {
            // 2. Transform the point from camera optical frame to the robot's base frame.
            //    "ursa_base_link" is the common base frame in your TF tree (Frame B).
            tf_buffer_.transform(point_in, point_out, "ursa_base_link");

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
               X_target_pt_.x() = point_out.point.x;
               X_target_pt_.y() = point_out.point.y;
               X_target_pt_.z() = point_out.point.z;
               target_received_ = true; // Mark as received
               ROS_WARN_STREAM("Target Re-acquired! Resetting filter.");
            } else {
               X_target_pt_.x() = alpha * point_out.point.x + (1.0 - alpha) * X_target_pt_.x();
               X_target_pt_.y() = alpha * point_out.point.y + (1.0 - alpha) * X_target_pt_.y();
               X_target_pt_.z() = alpha * point_out.point.z + (1.0 - alpha) * X_target_pt_.z();
            }

            last_target_time_ = ros::Time::now();
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

      // 2. Fetch Bottle Position (Ensure this is being sent in Frame B!)
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

      if (!is_data_fresh) {
        if (!holding_pos_latched_) {
            X_hold_ = X_tool_B; // Capture in Frame B
            holding_pos_latched_ = true;
        }
        Xd = X_hold_;
        ROS_WARN_STREAM_THROTTLE(1.0, "Holding Position: " << Xd << "\n"); 
      } else {
        holding_pos_latched_ = false; // Reset when tracking is active
        Xd = X_target_raw; 
        ROS_WARN_STREAM_THROTTLE(1.0, "Tracking Object. Target Position: " << Xd << "\n"); 
      }

      // Visualization: Direct in Frame B (ursa_base_link)
      publishMarker(Xd, 1.0, 0.0, 0.0, 1);
      publishMarker(X_tool_B, 0.0, 1.0, 0.0, 2);

      // 4. Calculate Cartesian Errors in Frame B
      cc::LinearPosition Delta_X_lin_B = X_tool_B - Xd;

      // Rotate Error to Frame 0 for Control Law
      // T_B_0 (Transform Points in 0 to Points in B). R_B_0 (Rotates from 0 to B).
      // We need to rotate FROM B TO 0. So we need R_B_0.inverse() or R_0_B.
      // model_.T_0_B() returns T_0_B (points 0 -> points B? No, T_0->B transform).
      // Let's rely on T_B_0().rotation() which is rotation of frame B expressed in 0?
      // Wait. Delta_X_0 = R_0_B * Delta_X_B.
      // R_0_B = model_.T_0_B().rotation().
      cc::LinearPosition Delta_X_lin_0 = model_.T_0_B().rotation().inverse() * Delta_X_lin_B;

      cc::CartesianVector Delta_X; 
      Delta_X.linear() = Delta_X_lin_0;
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
      
      // --- Velocity Saturation (Added) ---
      double max_vel = 0.5; // rad/s
      double scale = 1.0;
      double max_val = Qrp.lpNorm<Eigen::Infinity>(); // Max abs component
      if (max_val > max_vel) {
          scale = max_vel / max_val;
          Qrp = Qrp * scale;
      }

      Vector6d Qrpp;
      if (scale < 1.0) {
          // If saturated, assume constant velocity "cruise" -> zero acceleration
          // This avoids huge feedforward torques from trying to reach impossible velocities
          Qrpp = Vector6d::Zero();
      } else {
          // Normal acceleration feedforward
          Qrpp = J_pinv*(Xrpp - Jp_tool_0*Qrp);
      }
      // -----------------------------------

      // 6. Saturated Joint Space References
      JointState js_r;
      js_r = current; 
      js_r.qp = Qrp;
      js_r.qpp = Qrpp;

      // (Previous simple clamping Loop removed here)

      // 7. Slotine-Li Control
      Yr_ = model_.regressor(current.q, current.qp, js_r.qp, js_r.qpp);
      
      // Fix: Calculate Sq using the SATURATED Qrp. 
      // Previously: Sq = J_pinv * (Xp - Xrp) used the huge unsaturated Xrp.
      // Now: Sq = current.qp - Qrp (where Qrp is already saturated).
      Vector6d Sq = current.qp - Qrp;

      cc::Scalar gamma_inv = 1.0;
      cc::Scalar dt = 0.002;
      

      if (Sq.norm() > 1e-6) {
        theta_ -= gamma_inv * Yr_.transpose() * Sq * dt;
      } 

      tau = -Kd_ * Sq + Yr_ * theta_;
      ROS_WARN_STREAM_THROTTLE(1.0, "Cartesianspace Control \n");

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
      visualizeTrajectory(vQd);

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

      //cc::LinearPosition Delta_X_lin_0 = model_.T_0_B().rotation().inverse() * Delta_X_lin_B;

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
        //ROS_INFO_STREAM("Orientation O_lock_ in Base Link: " << O_lock_);
        //ROS_INFO_STREAM("theta_ right before switching: " << theta_);
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
