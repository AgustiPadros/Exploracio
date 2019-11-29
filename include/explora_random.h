#ifndef INCLUDE_TROBA_FRONTERES_H_
#define INCLUDE_TROBA_FRONTERES_H_

#include <ros/ros.h>
#include <vector>
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "exploracio/Fronteres.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "nav_msgs/GetPlan.h"

class ExploraRandom
{
    private:
      ros::NodeHandle nh_;
      ros::Publisher fronteres_pub_, mapa_fronteres_pub_, markers_pub_;
      ros::Subscriber fronteres_sub_, map_sub_;
      ros::ServiceClient get_plan_client_;

      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_move_base_;
      tf::TransformListener listener_;

      int robot_status_; //0: moving, 1: goal reached, 2: couldn't reach goal
      geometry_msgs::Pose target_goal_; //last sent goal
      int min_frontier_size_; //frontier size threshold
      geometry_msgs::Pose robot_pose_; //current robot position
      geometry_msgs::Pose prev_robot_pose_; //last robot pose, used to compute travelled distance

      nav_msgs::OccupancyGrid map_;
      exploracio::Fronteres fronteres_;

      // estadístiques
      int num_celles_; //number of map cells (height*width)
      int num_goals_enviats_; //total sent goals
      int num_goals_ok_; //succesfully reached goals
      double distancia_recorreguda_; //stores distance travelled by the robot wheels in meters
      ros::Time inici_exploracio_; //used to calculate elapsed time
      int celles_explorades_; //number of explored (!unknown) cells in the map
      bool exploracio_acabada_; //triggers end of exploration

    public:
      ExploraRandom(ros::NodeHandle& nh);
      void treballa();

    private:
      void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
      void fronteresCallback(const exploracio::Fronteres::ConstPtr& msg);

      // METODE
      bool replanifica();
      geometry_msgs::Pose decideixGoal();
      void acaba();

      // AUXILIARS
      geometry_msgs::Pose generaRandomPose(float radius);
      bool isValidGoal(const geometry_msgs::Point & point, double & path_length);
      double get_plan_length(std::vector<geometry_msgs::PoseStamped> poses);

      // LOCALITZACIÓ i NAVEGACIÓ
      bool moveRobot(const geometry_msgs::Pose& goal_pose);
      void move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result);
      void move_baseActive();
      bool refreshRobotPosition();
};

#endif /* INCLUDE_TROBA_FRONTERES_H_ */
