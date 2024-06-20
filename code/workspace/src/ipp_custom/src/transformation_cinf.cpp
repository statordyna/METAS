#include "../include/transformation.h"
#include <ipp_custom/getCostmap.h>
#include <ipp_custom/transferMap.h>

#include <cmath>
#include <ctime>

using namespace grid_map;

GridMap map({   "probability",
                "projection",
                "entropy",
                "transfer_map"
                });

GridMap Grid_Costmap({  "Entropy_Map",
                        "Inflation_Map",
                        "Costmap",
                        // "Frontier_Mask",
                        "Mask",
                        "Frontier_Map",
                        "Lethal_Cells",
                        "Unknown_Cells",
                        "Obstacle_Map",
                        "Ent_final"
                        });

std::vector<float> obs_time = {};
std::vector<float> fin_time = {};

std::vector<float> calculateSD(std::vector<float> data) {
  float sum = 0.0, mean, standardDeviation = 0.0;
  int i;

  for(i = 0; i < data.size(); ++i) {
    sum += data[i];
  }

  mean = sum / data.size();

  for(i = 0; i < data.size(); ++i) {
    standardDeviation += pow(data[i] - mean, 2);
  }

  return std::vector<float>{mean, sqrt(standardDeviation / data.size())};
}

bool METAS::getCostmapService(ipp_custom::getCostmap::Request &request, ipp_custom::getCostmap::Response &response){

    // std::cout<<"Get Costmap Service Called!"<<std::endl;

    while(!(flag_sub1 && flag_sub2 && flag_sub3)){
        ROS_WARN("WAITING FOR ALL SUBSCRIBER MESSAGES...");
        ros::Duration(2).sleep();
    }

    METAS::Calculate();

    response.costarray = costs_grid.data;
        
    METAS::Publish();
    return true;

}

float METAS::entropyTransformation(float m){

	if( m == 0.0 )	return 0.0;
	if( m == 1.0 )	return 0.0;
	if( m == -1.0 )	return 1.0;

	float e = -1 * ( m*log2(m) + (1-m)*log2(1-m) );
	return e;
}

void METAS::Update(){

    int i;
    
    i = ProbabilityMap.info.width*ProbabilityMap.info.height - 1;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it){
        map.at("probability", *it) = ProbabilityMap.data[i] >= 0 ? ProbabilityMap.data[i]/100.0 : ProbabilityMap.data[i];
        map.at("projection", *it) = ProjectionMap.data[i] >= 0 ? ProjectionMap.data[i]/100.0 : ProjectionMap.data[i];

        // map.at("mask", *it) = ProjectionMap.data[i] >= 0 && ProjectionMap.data[i] <= 100 ? 255.0 : 0.0;
        // map.at("mask_free", *it) = ProjectionMap.data[i] >= 0 && ProjectionMap.data[i] < 100 ? 255.0 : 0.0;
        // map.at("mask_obs", *it) = ProjectionMap.data[i] == 100 ? 255.0 : 0.0;
        
        map.at("entropy", *it) = ProbabilityMap.data[i] >= 0 ? METAS::entropyTransformation(ProbabilityMap.data[i]/100.0) : ProbabilityMap.data[i];
        i--;
    }

    i = CostmapMap.info.width*CostmapMap.info.height - 1;
    for (GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        Grid_Costmap.at("Inflation_Map", *it) = CostmapMap.data[i] >= 0 ? CostmapMap.data[i]/100.0 : -1;
        // Grid_Costmap.at("Frontier_Mask", *it) = CostmapMap.data[i] >= 0 ? (CostmapMap.data[i] <0.4*100 ? 255 : 0) : 0;
        i--;
    }
}

void METAS::Transfer(){
    
    float ent;
    int Idx_err_count = 0, Pos_err_count = 0;

    Grid_Costmap["Entropy_Map"].setConstant(-1);
    Grid_Costmap["Mask"].setConstant(-1);

    for(GridMapIterator it(map); !it.isPastEnd(); ++it){

        // ent = (map.at("transfer_map", *it)- min_E)/(max_E - min_E);
        ent = map.at("transfer_map", *it);
        Position pos;
        if(!map.getPosition(*it,pos)){
            // std::cout<<"ERROR: Position not found"<<std::endl;
            Pos_err_count++;
        }
        else{
            Index it_gc;
            if(!Grid_Costmap.getIndex(pos, it_gc)){
                // std::cout<<"ERROR: Index not found"<<std::endl;
                Idx_err_count++;
            }
            else{
                if(map.at("projection", *it) != -1){
                    Grid_Costmap.at("Entropy_Map", it_gc) = ent;
                    Grid_Costmap.at("Mask", it_gc) = 1;
                }
                else{
                    Grid_Costmap.at("Entropy_Map", it_gc) = -1;
                    Grid_Costmap.at("Mask", it_gc) = -1;
                }
            }
        }
    }
}

void METAS::Combine(){

    float inf;
    Grid_Costmap["Costmap"].setConstant(-1);
    for(GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        inf = Grid_Costmap.at("Inflation_Map", *it);
        int mask = Grid_Costmap.at("Mask", *it);
        if(mask == 0){
            Grid_Costmap.at("Ent_final", *it) = -1;
            Grid_Costmap.at("Unknown_Cells", *it) = 1;
        }
        else{
            Grid_Costmap.at("Ent_final", *it) = Grid_Costmap.at("Entropy_Map", *it);
            Grid_Costmap.at("Unknown_Cells", *it) = 0;
        }

//        if (inf>=0.95){
        if (Grid_Costmap.at("Obstacle_Map", *it) == -1){
            Grid_Costmap.at("Lethal_Cells", *it) = 1;
            Grid_Costmap.at("Ent_final", *it) = -1 ;
        }
        else
            Grid_Costmap.at("Lethal_Cells", *it) = 0;
    }
}

void METAS::Calculate(){

    int mapX = map.getSize().x(), mapY = map.getSize().y();
    int Grid_CostmapX = Grid_Costmap.getSize().x(), Grid_CostmapY = Grid_Costmap.getSize().y();
    int i;

    // New Method
    //
    ipp_custom::transferMap req;
    req.request.entropy_map.clear();
    req.request.projection_map.clear();

    for(GridMapIterator it(map); !it.isPastEnd(); ++it){
        req.request.entropy_map.push_back(100*map.at("entropy", *it));
        // req.request.entropy_map.push_back(map.at("entropy", *it));
        req.request.projection_map.push_back(map.at("projection", *it));
    }
    client2.call(req);

    i = 0;
    for(GridMapIterator it(map); !it.isPastEnd(); ++it){
        map.at("transfer_map", *it) = req.response.costmap[i];
        i++;
    }

    METAS::Transfer();

    //get Obstacle Map

    clock_t start = clock();
    for(GridMapIterator it(map); !it.isPastEnd(); ++it){
        if(Grid_Costmap.at("Inflation_Map", *it) >=0.95)
            Grid_Costmap.at("Obstacle_Map", *it) = -1;
        else
            Grid_Costmap.at("Obstacle_Map", *it) = 1;
    }
    float time_taken = float(clock() - start)/float(CLOCKS_PER_SEC);
    obs_time.push_back(time_taken);

    start = clock();
    METAS::Combine();
    time_taken = float(clock() - start)/float(CLOCKS_PER_SEC);
    fin_time.push_back(time_taken);

    std::cout<<"trial:\t"<<obs_time.size();
    std::vector<float> time1 = calculateSD(obs_time);
    std::vector<float> time2 = calculateSD(fin_time);
    std::cout<<"Obs time: "<<time1[0]<<" ; "<<time1[1]<<std::endl;
    std::cout<<"Fin time: "<<time2[0]<<" ; "<<time2[1]<<std::endl;


    std::vector<std::int16_t> costs_g(Grid_Costmap.getSize()(0) * Grid_Costmap.getSize()(1), 0);

    i = Grid_Costmap.getSize()(0) * Grid_Costmap.getSize()(1) - 1;
    GridCostmapMap.data.resize(i +1, 0);
    int max_val = -1;
    for (GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        std::int16_t value_ = Grid_Costmap.at("Ent_final", *it) >= 0.0 ? Grid_Costmap.at("Ent_final", *it) : -1;
        max_val = value_> max_val ? value_: max_val;
        costs_g[i] = value_;
        GridCostmapMap.data[i] = value_ >= 0.0 ? value_/7: -1;
        i--;
    }
    std::cout<<"GridMax: "<<max_val<<std::endl;
    costs_grid.data = costs_g;
}

void METAS::Publish(){

    CostmapArrayPublisher.publish(costs_grid);

    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    MapPublisher.publish(message);
    
    grid_map_msgs::GridMap message_;
    GridMapRosConverter::toMessage(Grid_Costmap, message_);
    CostmapPublisher.publish(message_);

    CostmapOccupancyPublisher.publish(GridCostmapMap);
}

METAS::~METAS() {}

METAS::METAS(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , spinner(4) {

    METAS::init_vars();
    METAS::init_maps();

    ROS_INFO("Starting Map Transformation Node.\n");

    sub1 = nh_.subscribe<nav_msgs::OccupancyGrid>("/rtabmap/grid_prob_map", 1, &METAS::ProbabilityMapCallback, this);
    sub3 = nh_.subscribe<nav_msgs::OccupancyGrid>("/rtabmap/proj_map", 1, &METAS::ProjectionMapCallback, this);
    sub4 = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &METAS::CostmapMapCallback, this);

    MapPublisher            = nh_.advertise<grid_map_msgs::GridMap>(Map_Gridmap_Topic, 1, true);
	CostmapPublisher        = nh_.advertise<grid_map_msgs::GridMap>(Costmap_Gridmap_Topic, 1, true);
	
    CostmapOccupancyPublisher = nh_.advertise<nav_msgs::OccupancyGrid>("/ipp_custom/costmap_occupancy_transformation", 1, true);
    CostmapArrayPublisher = nh_.advertise<std_msgs::Int16MultiArray>(Costmap_Array_Topic, 1, true);

    serviceClinet = nh_.advertiseService("/getCostmapSrv_transformation", &METAS::getCostmapService, this);
    client2 = nh_.serviceClient<ipp_custom::transferMap>("getCostmapTransfer");

    spinner.start();

    while(!(flag_sub1 && flag_sub2 && flag_sub3)){
        ROS_WARN("WAITING FOR ALL SUBSCRIBER MESSAGES...");
        ros::Duration(2).sleep();
    }

    while(ros::ok()){
        METAS::Update();
    }
}

void METAS::init_vars(){
    
    flag1 = 0;
    flag2 = 0;
    grid_init = 0;
    b_mx = 0, b_my = 0;

    if (!private_nh_.getParam("Costmap_Array_Topic", Costmap_Array_Topic)) {
        ROS_WARN("no param given to Costmap_Array_Topic");
        Costmap_Array_Topic = "/ipp_custom/costs_grid"; // Default Value
    }

    // --------------- Publishing Topics -----------------

    if (!private_nh_.getParam("Costmap_Gridmap_Topic", Costmap_Gridmap_Topic)) {
        ROS_WARN("no param given to Costmap_Gridmap_Topic");
        Costmap_Gridmap_Topic = "/ipp_custom/costmap_transformation"; // Default Value
    }
    if (!private_nh_.getParam("Map_Gridmap_Topic", Map_Gridmap_Topic)) {
        ROS_WARN("no param given to Map_Gridmap_Topic");
        Map_Gridmap_Topic = "/ipp_custom/map_transformation"; // Default Value
    }
    if (!private_nh_.getParam("Costmap_Image_Topic", Costmap_Image_Topic)) {
        ROS_WARN("no param given to Costmap_Image_Topic");
        Costmap_Image_Topic = "/ipp_custom/costmap_image"; // Default Value
    }
    if (!private_nh_.getParam("Frontier_Image_Topic", Frontier_Image_Topic)) {
        ROS_WARN("no param given to Frontier_Image_Topic");
        Frontier_Image_Topic = "/ipp_custom/frontier_image"; // Default Value
    }
    if (!private_nh_.getParam("Map_Image_Topic", Map_Image_Topic)) {
        ROS_WARN("no param given to Map_Image_Topic");
        Map_Image_Topic = "/ipp_custom/map_image"; // Default Value
    }
    if (!private_nh_.getParam("Obstacle_Image_Topic", Obstacle_Image_Topic)) {
        ROS_WARN("no param given to Obstacle_Image_Topic");
        Obstacle_Image_Topic = "/ipp_custom/obstacle_image"; // Default Value
    }

    // --------------- Costmap Parameters -----------------

    if (!private_nh_.getParam("costmap_x", cm_mx)) {
        ROS_WARN("no param given to Costmap Length X");
        cm_mx = 50.0; // Default Value
    }
    if (!private_nh_.getParam("costmap_y", cm_my)) {
        ROS_WARN("no param given to Costmap Length Y");
        cm_my = 50.0; // Default Value
    }
    if (!private_nh_.getParam("costmap_resolution", cm_res)) {
        ROS_WARN("no param given to costmap resolution");
        cm_res = 0.05; // Default Value
    }

    // --------------- Map Parameters -----------------

    if (!private_nh_.getParam("map_x", m_mx)) {
        ROS_WARN("no param given to Map Length X");
        m_mx = 31.05; // Default Value
    }
    if (!private_nh_.getParam("map_y", m_my)) {
        ROS_WARN("no param given to Map Length Y");
        m_my = 31.05; // Default Value
    }
    if (!private_nh_.getParam("map_resolution", m_res)) {
        ROS_WARN("no param given to Map resolution");
        m_res = 0.05; // Default Value
    }

    // --------------- Calculation Parameters -----------------

    if (!private_nh_.getParam("block_size", block_size)) {
        ROS_WARN("no param given to block_size");
        block_size = 10; // Default Value
    }
    if (!private_nh_.getParam("obstacle_dilution_size", obstacle_dilution_size)) {
        ROS_WARN("no param given to obstacle_dilution_size");
        obstacle_dilution_size = 20; // Default Value
    }
    if (!private_nh_.getParam("frontier_blur_radius", frontier_blur_radius)) {
        ROS_WARN("no param given to frontier_blur_radius");
        frontier_blur_radius = 5; // Default Value
    }
    if (!private_nh_.getParam("obstacle_blur_radius", obstacle_blur_radius)) {
        ROS_WARN("no param given to obstacle_blur_radius");
        obstacle_blur_radius = 3; // Default Value
    }
    if (!private_nh_.getParam("ptr1", ptr1)) {
        ROS_WARN("no param given to ptr");
        ptr1 = 400; // Default Value
    }
    if (!private_nh_.getParam("ptr2", ptr2)) {
        ROS_WARN("no param given to ptr");
        ptr2 = 700; // Default Value
    }
    if (!private_nh_.getParam("frontier_distance_limit", frontier_distance_limit)) {
        ROS_WARN("no param given to frontier_distance_limit");
        frontier_distance_limit = 6.0; // Default Value
    }
}

void METAS::init_maps(){

    map.setFrameId("map");
    map.setGeometry(Length(m_mx, m_mx),m_res);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells, resolution: %f).", map.getLength().x(), map.getLength().y(), map.getSize()(0), map.getSize()(1), ProbabilityMap.info.resolution);

    Grid_Costmap.setFrameId("map");
    Grid_Costmap.setGeometry(Length(cm_mx, cm_mx), cm_res);
    Grid_Costmap["Costmap"].setConstant(-1.0);
    ROS_INFO("Created Costmap with size %f x %f m (%i x %i cells, resolution: %f).", Grid_Costmap.getLength().x(), Grid_Costmap.getLength().y(), Grid_Costmap.getSize()(0), Grid_Costmap.getSize()(1), ProbabilityMap.info.resolution);

}

void METAS::ProbabilityMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // ROS_INFO("Probability Callback Received");
    flag_sub1 = 1;
    ProbabilityMap.header = msg->header;
    ProbabilityMap.info = msg->info;
    ProbabilityMap.data = msg->data;
}

void METAS::CostmapMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // ROS_INFO("Move_Base Costmap Callback Received");
    flag_sub2 = 1;
    b_mx = msg->info.origin.position.x;
    b_my = msg->info.origin.position.y;
    CostmapMap.data = msg->data;
    CostmapMap.info = msg->info;
    Grid_Costmap.setPosition(Position(25.0 + b_mx, 25.0 + b_my));
    GridCostmapMap.info = msg->info;
    // GridCostmapMap.data = msg->data;
}

void METAS::ProjectionMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // ROS_INFO("Projection Callback Received");
    flag_sub3 = 1;
    ProjectionMap.header = msg->header;
    ProjectionMap.info = msg->info;
    ProjectionMap.data = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transformation_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    METAS transformationNode(nh, private_nh);
    return 0;
}
