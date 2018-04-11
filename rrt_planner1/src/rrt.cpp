#include "rrt.h"
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include<cstdlib>
#include <string>
#include<stdlib.h>
#include <cmath>
#include "boost/bind.hpp"
#include <pluginlib/class_list_macros.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
PLUGINLIB_EXPORT_CLASS(rrt_plan1::rrt_planner1, nav_core::BaseGlobalPlanner)
bool* OGM;
namespace rrt_plan1
{
    rrt_planner1::rrt_planner1():initialized_(false)
    {

    }
    rrt_planner1::rrt_planner1(ros::NodeHandle &nh)
    {
        ROSNodeHandle = nh;
    }

    rrt_planner1::rrt_planner1(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }
    void rrt_planner1::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {   

        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            int mapSize = width*height;

            ROS_INFO("---originX=%f,originY=%f---",originX,originY);

            ROS_INFO("---width=%d,height=%d---",width,height);

            ROS_INFO("---resolution=%f---",resolution);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
       
            OGM = new bool [mapSize];
            for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
            {
                for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                if(cost<=78)  
                    OGM[iy*width+ix]=true;
                else
                    OGM[iy*width+ix]=false;
                }
            }        
            ROS_INFO("---rrt planner1 initialized successfully---");
            initialized_ = true;
        }
        else
        {
            ROS_WARN("---This planner1 has already been initialized... doing nothing---");
        }
    }
    bool rrt_planner1::isStateValid( const ob::State *state)
    {
          const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
          const double &x(state_2d->getX()), &y(state_2d->getY());
          if(OGM[getCellIndex(x,y)])
              return true;
          else
              return false;         
    }
    bool rrt_planner1::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
    {
            if (!initialized_)
            {
                ROS_ERROR("---The planner has not been initialized, please call initialize() to use the planner---");
                return false;
            }
            if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
            {
                ROS_ERROR("---This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.---",
                costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
                return false;
            }
            plan.clear();
            tf::Stamped < tf::Pose > goal_tf;
            tf::Stamped < tf::Pose > start_tf;

            poseStampedMsgToTF(goal, goal_tf);
            poseStampedMsgToTF(start, start_tf);
            float startX = start.pose.position.x;
            float startY = start.pose.position.y;

            float goalX = goal.pose.position.x;
            float goalY = goal.pose.position.y;

            getCorrdinate(startX, startY);
            getCorrdinate(goalX, goalY);

            int startCell;
            int goalCell;

            startCell = convertToCellIndex(startX, startY);
            goalCell = convertToCellIndex(goalX, goalY);

            ob::StateSpacePtr space(new ob::SE2StateSpace());
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-400);
            bounds.setHigh(400);
            space->as<ob::SE2StateSpace>()->setBounds(bounds);

            og::SimpleSetup ss(space);

            ss.setStateValidityChecker(boost::bind(&rrt_planner1::isStateValid,this, _1));

            ob::ScopedState<ob::SE2StateSpace> startpl(space);
            startpl->setXY(getCellRowID(startCell), getCellColID(startCell));

            ob::ScopedState<ob::SE2StateSpace> goalpl(space);
            goalpl->setXY(getCellRowID(goalCell), getCellColID(goalCell));

            ss.setStartAndGoalStates(startpl, goalpl);
            ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));

            ss.setPlanner(planner);
            ob::PlannerStatus solved = ss.solve(3.0);
            if (solved)
            {
                ss.getSolutionPath().print(cout);
                std::ofstream ofs("/home/ljq/path.dat");
                ss.getSolutionPath().printAsMatrix(ofs);
            }
            else
            {
                ROS_ERROR("Failed to determine plan");
            }
                fstream file("/home/ljq/path.dat");
                vector< vector<float> >path;
                while (file.good())
                {       
                    for (int i=0;i<1000;i++)
                    {
                        vector<float>temp;
                        temp.clear();
                        float grid[3];
                        for(int j=0;j<3;j++)
                        {                               
                                file>>grid[j];
                                temp.push_back(grid[j]);                               
                        }
                        path.push_back(temp);
                }
            
            path.erase(unique(path.begin(),path.end()),path.end());
            
            for(int it=0;it<path.size();it++)
            {
                float x=0.0,y=0.0;
                int cellIndex=getCellIndex(path[it][0],path[it][1]);
                convertToCoordinate(cellIndex, x, y);
              
                geometry_msgs::PoseStamped pose = goal;


                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);

            }
                    plan.push_back(goal);
            }
         
            publishPlan(plan);
            return !plan.empty();
    }
    void rrt_planner1::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if (!initialized_) 
        {
                ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if (!path.empty()) 
        {
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

   
        for (unsigned int i = 0; i < path.size(); i++) 
        {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
}