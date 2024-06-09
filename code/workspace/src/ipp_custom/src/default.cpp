#include "../include/transformation.h"
#include <ipp_custom/getCostmap.h>

#include <cmath>
#include <ctime>

using namespace grid_map;

GridMap map({   "probability",
                "projection",
                "entropy",
                "entropy_map",
                "mask",
                "mask_obs",
                "mask_free"
                });

GridMap Grid_Costmap({  "Entropy_Map",
                        "Inflation_Map",
                        "Costmap",
                        "Frontier_Mask",
                        "Mask",
                        "Frontier_Map",
                        "Lethal_Cells",
                        "Unknown_Cells"
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

    std::cout<<"Get Costmap Service Called!"<<std::endl;

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

void METAS::averageEntropy()
{
    double value{NAN};
    double radius = 0.15;

    //Taken from the official grid_map mean filter implementation
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        double valueSum = 0.0;
        int counter = 0;
        
        Eigen::Vector2d center;
        map.getPosition(*iterator, center);

        for (grid_map::CircleIterator submapIterator(map, center, radius); !submapIterator.isPastEnd(); ++submapIterator) {
            if (!map.isValid(*submapIterator, "entropy_map")) {
                continue;
            }
            value = map.at("entropy", *submapIterator);
            valueSum += value;
            counter++;
        }

        if (counter != 0) {
            map.at("entropy_map", *iterator) = valueSum / counter;
        }
        else{
            map.at("entropy_map", *iterator) = 1;
        }
    }
}


void METAS::Update(){

    int i;
    
    i = ProbabilityMap.info.width*ProbabilityMap.info.height - 1;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it){
        map.at("probability", *it) = ProbabilityMap.data[i] >= 0 ? ProbabilityMap.data[i]/100.0 : ProbabilityMap.data[i];
        map.at("projection", *it) = ProjectionMap.data[i] >= 0 ? ProjectionMap.data[i]/100.0 : ProjectionMap.data[i];

        map.at("mask", *it) = ProjectionMap.data[i] >= 0 && ProjectionMap.data[i] <= 100 ? 255.0 : 0.0;
        map.at("mask_free", *it) = ProjectionMap.data[i] >= 0 && ProjectionMap.data[i] < 100 ? 255.0 : 0.0;
        map.at("mask_obs", *it) = ProjectionMap.data[i] == 100 ? 255.0 : 0.0;
        
        map.at("entropy", *it) = ProbabilityMap.data[i] >= 0 ? METAS::entropyTransformation(ProbabilityMap.data[i]/100.0) : ProbabilityMap.data[i];
        i--;
    }

    i = CostmapMap.info.width*CostmapMap.info.height - 1;
    for (GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        Grid_Costmap.at("Inflation_Map", *it) = CostmapMap.data[i] >= 0 ? CostmapMap.data[i]/100.0 : -1;
        Grid_Costmap.at("Frontier_Mask", *it) = CostmapMap.data[i] >= 0 ? (CostmapMap.data[i] <0.4*100 ? 255 : 0) : 0;
        i--;
    }
}

void METAS::Transfer(float max_E ,float max_F ,float max_I ,float min_E ,float min_F ,float min_I){
    
    float ent;
    int Idx_err_count = 0, Pos_err_count = 0;

    Grid_Costmap["Entropy_Map"].setConstant(-1);
    for(GridMapIterator it(map); !it.isPastEnd(); ++it){

        ent = (map.at("entropy_map", *it)- min_E)/(max_E - min_E);
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
                }
                else{
                    Grid_Costmap.at("Entropy_Map", it_gc) = -1;
                }
            }
        }
    }

    // if(Idx_err_count != 0) std::cout<<"Index Error count:\t"<<Idx_err_count<<std::endl;
    // if(Pos_err_count != 0) std::cout<<"Position Error count:\t"<<Pos_err_count<<std::endl;

}

void METAS::Combine(){

    float ent, frn, inf;
    float alpha, beta;
    Grid_Costmap["Costmap"].setConstant(-1);

    for(GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        inf = Grid_Costmap.at("Inflation_Map", *it);
        if(inf < 0){
            Grid_Costmap.at("Costmap", *it) = -1;
            Grid_Costmap.at("Unknown_Cells", *it) = 1;
        }
        else{
            beta = 1000;
            Grid_Costmap.at("Costmap", *it) = inf*beta;
            Grid_Costmap.at("Unknown_Cells", *it) = 0;
        }

        if (Grid_Costmap.at("Costmap", *it)>=850)
            Grid_Costmap.at("Lethal_Cells", *it) = 1;
        else
            Grid_Costmap.at("Lethal_Cells", *it) = 0;
    }
}

void METAS::Calculate(){

    int mapX = map.getSize().x(), mapY = map.getSize().y();
    int Grid_CostmapX = Grid_Costmap.getSize().x(), Grid_CostmapY = Grid_Costmap.getSize().y();

    // // Get Frontier Costs
    // cv::Mat Mask, binaryMask, resMask, RszImg;
    // GridMapCvConverter::toImage<unsigned char , 1>(Grid_Costmap, "Frontier_Mask", CV_8U, 0.0, 255.0, Mask);
    // cv::resize(Mask, RszImg, cv::Size(200, 200));
    // cv::distanceTransform(RszImg, resMask, cv::DIST_L2, 5);
    // cv::resize(resMask, resMask, cv::Size(Grid_CostmapX, Grid_CostmapY), 0,0,cv::INTER_LINEAR);
    // GridMapCvConverter::addLayerFromImage<float, 1>(resMask, "Frontier_Map", Grid_Costmap, 0.0, 1.0);

    // int i = ProbabilityMap.info.width*ProbabilityMap.info.height - 1;

    // float max_E = -1, max_F = -1, max_I = -1, min_E = 1000, min_F = 1000, min_I = 1000;
    // METAS::averageEntropy();
    // for (GridMapIterator it(map); !it.isPastEnd(); ++it){
    //     float epb_val, inf_val, frn_val;
    //     epb_val = map.at("entropy_map", *it);
    //     if(epb_val > max_E) max_E = epb_val;
    //     if(epb_val < min_E && epb_val != -1) min_E = epb_val;
    // }

    // METAS::Transfer(max_E , max_F , max_I , min_E , min_F , min_I);
    clock_t start = clock();
    METAS::Combine();
    float time_taken = float(clock() - start)/float(CLOCKS_PER_SEC);
    fin_time.push_back(time_taken);

    std::cout<<"trial:\t"<<fin_time.size();
    std::vector<float> time2 = calculateSD(fin_time);
    std::cout<<"Fin time: "<<time2[0]<<" ; "<<time2[1]<<std::endl;

    std::vector<std::int16_t> costs_g(Grid_Costmap.getSize()(0) * Grid_Costmap.getSize()(1), 0);

    int i = Grid_Costmap.getSize()(0) * Grid_Costmap.getSize()(1) - 1;
    GridCostmapMap.data.resize(i +1, 0);
    for (GridMapIterator it(Grid_Costmap); !it.isPastEnd(); ++it){
        std::int16_t value_ = Grid_Costmap.at("Costmap", *it) >= 0.0 ? Grid_Costmap.at("Costmap", *it) : -1;
        costs_g[i] = value_;
        GridCostmapMap.data[i] = value_ >= 0.0 ? value_/10: -1;
        i--;
    }

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

    // cv::Mat ent_img;
    // GridMapCvConverter::toImage<unsigned short, 1>(map, "entropy_map", CV_16UC1, -1.0, 1.0, ent_img);

    // std_msgs::Header header;

    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, ent_img);
    // img_bridge.toImageMsg(mapImg);
    // MapImagePublisher.publish(mapImg);
    
}

METAS::~METAS() {}

METAS::METAS(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh)
    , private_nh_(private_nh)
    , spinner(4) {

    METAS::init_vars();
    METAS::init_maps();

    ROS_INFO("Starting Map Transformation Default Node.\n");

    sub1 = nh_.subscribe<nav_msgs::OccupancyGrid>("/rtabmap/grid_prob_map", 1, &METAS::ProbabilityMapCallback, this);
    sub3 = nh_.subscribe<nav_msgs::OccupancyGrid>("/rtabmap/proj_map", 1, &METAS::ProjectionMapCallback, this);
    sub4 = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1, &METAS::CostmapMapCallback, this);

    MapPublisher            = nh_.advertise<grid_map_msgs::GridMap>(Map_Gridmap_Topic, 1, true);
	CostmapPublisher        = nh_.advertise<grid_map_msgs::GridMap>(Costmap_Gridmap_Topic, 1, true);
	
    CostmapOccupancyPublisher = nh_.advertise<nav_msgs::OccupancyGrid>("/ipp_custom/costmap_occupancy_default", 1, true);
    CostmapArrayPublisher = nh_.advertise<std_msgs::Int16MultiArray>(Costmap_Array_Topic, 1, true);

    serviceClinet = nh_.advertiseService("/getCostmapSrv_default", &METAS::getCostmapService, this);

    spinner.start();

    while(!(flag_sub1 && flag_sub2 && flag_sub3)){
        ROS_WARN("WAITING FOR ALL SUBSCRIBER MESSAGES...");
        ros::Duration(2).sleep();
    }

    // while(ros::ok()){

    //     spinner.stop();
    //     METAS::Update();
    //     METAS::Calculate();
    //     METAS::Publish();
    //     spinner.start();
    // }
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
        Costmap_Array_Topic = "/ipp_custom/costs_grid_default"; // Default Value
    }

    // --------------- Publishing Topics -----------------

    if (!private_nh_.getParam("Costmap_Gridmap_Topic", Costmap_Gridmap_Topic)) {
        ROS_WARN("no param given to Costmap_Gridmap_Topic");
        Costmap_Gridmap_Topic = "/ipp_custom/costmap_default"; // Default Value
    }
    if (!private_nh_.getParam("Map_Gridmap_Topic", Map_Gridmap_Topic)) {
        ROS_WARN("no param given to Map_Gridmap_Topic");
        Map_Gridmap_Topic = "/ipp_custom/map_default"; // Default Value
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
    ros::init(argc, argv, "default_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    METAS transformationNode(nh, private_nh);
    return 0;
}