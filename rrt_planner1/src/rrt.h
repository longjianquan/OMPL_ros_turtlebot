#ifndef RRT_H
#define RRT_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/foreach.hpp>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>


#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#include <iostream>
#include <functional>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <time.h>

#include <set>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>




namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
using namespace std;
using std::string;
namespace rrt_plan1
{
class rrt_planner1 : public nav_core::BaseGlobalPlanner
{
public:
    rrt_planner1 (ros::NodeHandle &); 
    rrt_planner1 ();
    rrt_planner1(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ros::NodeHandle ROSNodeHandle;

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    
    
    bool isStartAndGoalCellsValid(int startCell,int goalCell);
    bool isStateValid( const ob::State *state);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);


    void getCorrdinate (float &x, float &y)
    {
        x = x - originX;
        y = y - originY;
    }

    int getCellIndex(int i,int j)//根据cell所处的行和列得到cell的索引值
    {
        return (i*width)+j;
    }

    int getCellRowID(int index)//根据cell的索引值得到行数
    {
        return index/width;
    }

    int getCellColID(int index)//根据索引值得到列数
    {
        return index%width;
    }

    int convertToCellIndex (float x, float y)//根据坐标值得到cell索引值
    {
        int cellIndex;

        float newX = x / resolution;
        float newY = y / resolution;

        cellIndex = getCellIndex(newY, newX);

        return cellIndex;
    }

    void convertToCoordinate(int index, float &x, float &y)//根据cell索引值得到坐标值
    {
        x = getCellColID(index) * resolution;

        y = getCellRowID(index) * resolution;

        x = x + originX;
        y = y + originY;
    }
    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS* costmap_ros_;    
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::CostmapModel* _costmap_model;
    bool initialized_;
    int width;
    int height;

    ros::Publisher plan_pub_;

};
};
#endif