#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <std_msgs/Int8.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>


class blockTransformation{

    public:

        blockTransformation(ros::NodeHandle nh, ros::NodeHandle private_nh);
        ~blockTransformation();

    private:

        void init_vars();
        void init_maps();

        ros::NodeHandle nh_, private_nh_;
        ros::AsyncSpinner spinner;

        // TODO:
        //      For some reason GridMap isn't allowing the maps to be declared here.. check on that later..
        //      Change the Float Arr to Unsigned Short Int Arr.. (less space)
        //
        // grid_map::GridMap map({"probability", "projection", "projection_diluted", "inflated_costs", "entropy", "frontiers", "frontier_cost", "frontier_cost_map", "entropy_blocks", "combination", "combination_scaled"});
        // grid_map::GridMap Grid_Costmap({"Costmap", "Costmap_scaled"});

        // std_msgs::Float32MultiArray costs_grid;
        std_msgs::Int16MultiArray costs_grid;

        nav_msgs::OccupancyGrid ProbabilityMap;
        nav_msgs::OccupancyGrid ProjectionMap;
        nav_msgs::OccupancyGrid FrontierMap;
        nav_msgs::OccupancyGrid CostmapMap;

        ros::Subscriber sub1;
        ros::Subscriber sub2;
        ros::Subscriber sub3;
        ros::Subscriber sub4;

        int flag_sub1  = 0, flag_sub2  = 0, flag_sub3  = 0, flag_sub4  = 0;
        sensor_msgs::Image mapImg, ObsImage, FrontierImage;
        cv_bridge::CvImage img_bridge;

        ros::Publisher MapPublisher;
        ros::Publisher CostmapPublisher;

        ros::Publisher MapImagePublisher;
        ros::Publisher CostmapImagePublisher;
        ros::Publisher FrontierImagePublisher;
        ros::Publisher ObstacleImagePublisher;

        ros::Publisher CostmapArrayPublisher;

        void ProbabilityMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void CostmapMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void ProjectionMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void FrontierCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        void Update();
        void Calculate();
        void Publish();

        float entropyTransformation(float m);
        void submapTransformation();
        void combineMaps(float max_E ,float max_F ,float max_I ,float min_E ,float min_F ,float min_I);
        
        int flag1;
        int flag2;
        // int block_size = 10;
        // float map_x;
        // float map_y;
        int grid_init;
        float b_mx, b_my; // Map position in world;


        // ROS Parameter Variables :

        // Defining Publishing Topics names as parameters cause
        // we'll be running multiple Maps at one time.
        std::string Costmap_Array_Topic;
        std::string Costmap_Gridmap_Topic, Map_Gridmap_Topic; 
        std::string Costmap_Image_Topic, Map_Image_Topic, Frontier_Image_Topic, Obstacle_Image_Topic;
        
        // Costmap Parameter
        float cm_mx, cm_my; // Length of Costmap (in meters)
        float cm_res; // Resolution of Costmap (in meters)

        // Map Parameter
        float m_mx, m_my; // Length of Map (in meters)
        float m_res; // Resolution of Map (in meters)
        
        // Transformation Parameters
        int block_size; // Block Size for Entropy Blocks
        int obstacle_dilution_size; // 
        int obstacle_scaling_factor; // 
        int frontier_blur_radius; //
        int obstacle_blur_radius; //
        int ptr1, ptr2;
        cv::Mat inflate_elem;

};

// TODO : Exponential Inflation Method