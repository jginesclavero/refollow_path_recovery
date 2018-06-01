#include <refollow_path_recovery/refollow_path_recovery.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(refollow_path_recovery::RefollowPathRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace refollow_path_recovery {
  RefollowPathRecovery::RefollowPathRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

  void RefollowPathRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
      if(!initialized_){
        ros::NodeHandle private_nh("~/" + name_);
        ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");
        name_ = name;
        tf_ = tf;
        global_costmap_ = global_costmap;
        local_costmap_ = local_costmap;

        private_nh.param("frequency", frequency_, 10.0);
        private_nh.param("sim_granularity", sim_granularity_, 0.017);

        blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
        blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
        blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
        blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
        initialized_ = true;
        orientation_ready = false;
        turning = false;
        aborting = false;
      }else
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");

    }

    void RefollowPathRecovery::pathCallback(const nav_msgs::Path::ConstPtr& path){
      tf::quaternionMsgToTF(path->poses[5].pose.orientation, orientation_);
      ROS_INFO("orientation ready");
      orientation_ready = true;
    }

    void RefollowPathRecovery::timerCallback(const ros::TimerEvent&){
      if (!orientation_ready)
        aborting = true;
    }

    void RefollowPathRecovery::runBehavior(){
      if(!initialized_){
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
      }

      if(global_costmap_ == NULL || local_costmap_ == NULL){
        ROS_ERROR("The costmaps passed to the RefollowPathRecovery object cannot be NULL. Doing nothing.");
        return;
      }
      ROS_WARN("RefollowPathRecovery behavior started");

      ros::Rate r(frequency_);
      ros::NodeHandle n;
      ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      ros::Subscriber path_sub = n.subscribe("/move_base/GlobalPlanner/plan", 1, &RefollowPathRecovery::pathCallback,this);
      tf::Stamped<tf::Pose> global_pose;

      local_costmap_->getRobotPose(global_pose);
      double current_angle = -1.0 * M_PI;
      double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
      timer	=	n.createTimer(ros::Duration(3),&RefollowPathRecovery::timerCallback,this,true);

      while(n.ok()){
        ROS_WARN("While");
        if (orientation_ready){
          global_costmap_->getRobotPose(global_pose);
          double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
          current_angle = angles::normalize_angle(norm_angle + start_offset);
          double orientation_yaw = angles::normalize_angle(tf::getYaw(orientation_));
          ROS_WARN("orientation_yaw  %f",orientation_yaw);
          ROS_WARN("current_angle %f",current_angle);
          double dist_left = fabs(orientation_yaw - current_angle);
          ROS_WARN("dist left %f",dist_left);
          if(turning && dist_left < 0.2){
            ROS_WARN("stop send velocities");
            return;
          }

          if(dist_left > 2.5 || turning){
              ROS_WARN("sending velocities");
              turning = true;
              geometry_msgs::Twist cmd_vel;
              cmd_vel.linear.x = 0.0;
              cmd_vel.linear.y = 0.0;
              cmd_vel.angular.z = 0.2;
              vel_pub.publish(cmd_vel);
          }
        }

        if(aborting)
          return;

        ros::spinOnce();
        r.sleep();
      }
    }
  };
