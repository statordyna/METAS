#include "../include/three_map_fsm_node.h"
#include <robotino_fsm/GetPlan_.h>
#include "robotino_fsm/getCostmap_.h"

double dist(std::vector<double> start, std::vector<double> end) {
    return std::pow((start[0] - end[0]) * (start[0] - end[0]) +
                    (start[1] - end[1]) * (start[1] - end[1]) +
                    (start[2] - end[2]) * (start[2] - end[2]), 0.5);
}

void getRPY(geometry_msgs::Pose odom, double &roll, double &pitch, double &yaw) {
    tf::Quaternion q(
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

// // NEW ADDITION: callback for costs_grid
void robotino_fsm_node::getCostsGrid(const std_msgs::Int16MultiArray::ConstPtr &msg){
    for(int i = 0; i < 1000*1000; i++){
        costs_grid[i] = msg->data[i];
    }
    // std::cout<<"GOT COSTS_GRID";
}
void robotino_fsm_node::getCostsGridDefault(const std_msgs::Int16MultiArray::ConstPtr &msg){
    for(int i = 0; i < 1000*1000; i++){
        costs_grid_default[i] = msg->data[i];
    }
    // std::cout<<"GOT COSTS_GRID";
}

void robotino_fsm_node::mpcStateCallback(const std_msgs::Int8::ConstPtr &msg) {
    emergency = false;

    // if ok and goal available go on.
    // if ok and goal NOT available require new goal
    if ((msg->data == REACHED || msg->data == WAITING) && !send_goal) {
        filter_last = false;
        // empty or last element (reached the end)
        if (currentPlan.size() <= 1) {
            currentPlan.clear();
            require_new_goal = true;
            send_goal = false;
        } else {
            // send next goal
            currentPlan.erase(currentPlan.begin());
            require_new_goal = false;
            send_goal = true;
        }
        replan = false;
        failed = 0;
        failed_replans = 0;
        if (is_second == 0)
            replan_cnt = 0;
        else {
            is_second -= 1;
        }
        failed_plans = 0;
        counter_lc += 1;
    }
        // if moving / operating / computing do not send goal / do anything
    else if (msg->data == OPERATING || msg->data == RECEIVED) {
        require_new_goal = false;
        replan = false;
        send_goal = false;
        filter_last = false;
        // reset only while operating
    }
        // here recoveries, we're either oscillating or getting invalid state from the mpc
    else if (msg->data == OSCILLATING || msg->data == INVALID) {

        ROS_WARN_STREAM(
                "ROBOT IS " << (msg->data == OSCILLATING ? " OSCILLATING " : " INVALID") << ". Number failures = "
                            << failed);
        // basic cleaning to have a clean state to avoid artifacts (moving obstacles and similar that might have gone away)
        {    // disable obstacles
            dynamic_reconfigure::Reconfigure conf;
            dynamic_reconfigure::DoubleParameter param;
            param.name = "cluster_max_distance";
            param.value = 0;
            conf.request.config.doubles.emplace_back(param);
            clear_obstacles.call(conf);

            // clear costmap
            std_srvs::Empty empty;
            clear_costmap.call(empty);

            // re enable obstacles
            param.value = 0.4;
            conf.request.config.doubles.clear();
            conf.request.config.doubles.emplace_back(param);
            clear_obstacles.call(conf);
            ros::Duration(3).sleep();
        }

        // count the failed attempt
        failed += 1;

        // if < than threshold try to recover (replan or move to next goal) otherwise require new goal
        if (failed < max_fail_before_new_goal) {
            // again we could be essentially at the last goal (and oscillating nearby),
            // hence our scope now is to get a new plan for a new frontier point.
            // Therefore if we're at last goal, we want to avoid that as next goal (filter last) and require new goal
            if (currentPlan.size() <= 1) {
                ROS_INFO_STREAM("plan already done. Getting a new goal");
                require_new_goal = true;
                filter_last = true;
                send_goal = false;
            } else
                ROS_INFO_STREAM("plan with possible solution. Size " << currentPlan.size());
            // if oscillating near the next goal try to go on to the following one
            if (msg->data == OSCILLATING &&
                dist(std::vector<double>{globalRobot.position.x, globalRobot.position.y, globalRobot.position.z},
                     std::vector<double>{currentPlan.begin()->position.x, currentPlan.begin()->position.y,
                                         currentPlan.begin()->position.z}) < 0.5) {
                ROS_INFO_STREAM("Delete current and go on.");
                // send next goal
                if (!currentPlan.empty())
                    currentPlan.erase(currentPlan.begin());
                // at this point dist to the next point > 0.5 because of how we have defined wp
                send_goal = !currentPlan.empty();
                require_new_goal = currentPlan.empty();
                replan = false;
            } else {
                // at this point the planning will be either if not oscillating or if the distance is great
                replan = true;
                if (replan_cnt > 3) {
                    require_new_goal = true;
                    replan = false;
                    filter_last = true;
                }
            }
        } else {
            if (filter_last) {
                emergency = true;
            } else {
                // if everything fail require new goal
                filter_last = true;
                require_new_goal = true;
            }
        }
    }
}

void robotino_fsm_node::rtabmapInfoCallback(const rtabmap_ros::Info::ConstPtr &msg) {
    if (msg->loopClosureId > 0 || msg->proximityDetectionId > 0) {
        loop_clos.emplace_back(msg->loopClosureId > 0 ? msg->loopClosureId : msg->proximityDetectionId);
        //   replan = true;
    }
}

void robotino_fsm_node::general_reset() {
    //stop MPC while we try to recover and reset
    nav_msgs::Odometry goal;
    goal.header.frame_id = "emergency";
    goal.pose.pose = globalRobot;
    mpcGoalPub.publish(goal);

    //stop the robot
    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    pubCamCmd.publish(msg);
//    msg.angular.z = 0;
//    pubRobCmd.publish(msg);

    // pause rtabmap and reset odometries [icp + camera + robot] we do NOT care much about poses
    // (will be rectified. The EKFs work in the speed domain apart from the LC).
    ros::Duration(2).sleep();
    std_srvs::Empty empty_srv;
    rtabmap_pause.call(empty_srv);

    rtabmap_resetodom.call(empty_srv);

    rtabmap_newmap.call(empty_srv);

    robot_localization::SetPose pose;
    pose.request.pose.header.frame_id = "map";
    pose.request.pose.pose.pose.position = globalRobot.position;
    pose.request.pose.pose.pose.orientation = globalRobot.orientation;
    ekf_robot_reset.call(pose);


    // rotate the camera WHILE mapping (try trigger LC + clean obstacles)
    msg.angular.z = 0.6;
    pubRobCmd.publish(msg);
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.angular.z = 0;
//    pubRobCmd.publish(msg);
    ros::Duration(1).sleep();

    rtabmap_resume.call(empty_srv);
    ros::Duration(12).sleep(); // total sleep time = 10 s == 2 PI camera turning. The MPC will take over.

    replan_cnt = 0;
    failed_plans = 0;
    failed_replans = 0;
    failed = 0;
    replan = false;
    require_new_goal = true;
    emergency = false;
}

robotino_fsm_node::~robotino_fsm_node() {}

robotino_fsm_node::robotino_fsm_node(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh),
                                                                                       private_nh_(private_nh),
                                                                                       spinner(4) {

    robotino_fsm_node::init_vars();

    get_frontier = nh_.serviceClient<active_slam::get_frontier_list>(get_frontier_topic);
    // get odom pose + rotations at the same time from both cam and base
    message_filters::Subscriber<nav_msgs::Odometry> odomRobotSub(nh_, odom_robot_topic, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odomCamSub(nh_, odom_cam_topic, 1);
    if (sim) {
        exact.reset(new sync_exact(MyExactSyncPolicy (10), odomRobotSub, odomCamSub));
        exact->registerCallback(boost::bind(&robotino_fsm_node::odomCallback,this, _1, _2));
    }
    else{
        approx.reset(new sync_approx(MyApproxSyncPolicy (10), odomRobotSub, odomCamSub));
        approx->registerCallback(boost::bind(&robotino_fsm_node::odomCallback,this, _1, _2));
    }

    // get mpc state real time
    mpcStateClient = nh_.subscribe<std_msgs::Int8>(mpcStateTopic, 1, &robotino_fsm_node::mpcStateCallback, this);

    rtabmap_info = nh_.subscribe<rtabmap_ros::Info>("/rtabmap/info", 1, &robotino_fsm_node::rtabmapInfoCallback, this);

    // NEW ADDITION: Get costs_grid
    subCostsGrid = nh_.subscribe<std_msgs::Int16MultiArray>("/ipp_custom/costs_grid", 1, &robotino_fsm_node::getCostsGrid, this);
    subCostsGridDefault = nh_.subscribe<std_msgs::Int16MultiArray>("/ipp_custom/costs_grid_default", 1, &robotino_fsm_node::getCostsGridDefault, this);

    // publish waypoints
    mpcGoalPub = nh_.advertise<nav_msgs::Odometry>(mpcGoalTopic, 1);

    //NEW ADDITION
    pubCurrentPlanPath = nh_.advertise<nav_msgs::Path>("/current_plan_path", 1);

    pubC2DPlanPath = nh_.advertise<nav_msgs::Path>("/costmap_2d_plan_viz", 1);
    pubGMPlanPath = nh_.advertise<nav_msgs::Path>("/gridmap_plan_viz", 1);
    pubTRNPlanPath = nh_.advertise<nav_msgs::Path>("/transformation_plan_viz", 1);
    
    pubDefaultCurrentPlanPath = nh_.advertise<nav_msgs::Path>("/default_plan_path", 1);
    pubCurrentPaths = nh_.advertise<nav_msgs::Path>("/current_paths", 4);
    pubDefaultPaths = nh_.advertise<nav_msgs::Path>("/default_paths", 4);
    pubGridmapPaths = nh_.advertise<nav_msgs::Path>("/gridmap_paths", 4);
    defaultPathReq_client = nh_.serviceClient<robotino_fsm::GetPlan_>(get_default_plan_topic);
    pubGroupedCurrentPath = nh_.advertise<active_slam::GroupPathInfo>("/grouped_current_paths_info", 1);
    pubGroupedDefaultPath = nh_.advertise<active_slam::GroupPathInfo>("/grouped_default_paths_info", 1);

    pubGroupedCostmap2DPath = nh_.advertise<active_slam::GroupPathInfo>("/grouped_costmap2D_paths_info", 1);;
    pubGroupedGridmapDefaultPath = nh_.advertise<active_slam::GroupPathInfo>("/grouped_gridmap_default_paths_info", 1);;
    pubGroupedGridmapTransformationPath = nh_.advertise<active_slam::GroupPathInfo>("/grouped_gridmap_transformation_paths_info", 1);;

    getCostmapClientTransformation = nh_.serviceClient<robotino_fsm::getCostmap_>("/getCostmapSrv_transformation");
    getCostmapClientDefault = nh_.serviceClient<robotino_fsm::getCostmap_>("/getCostmapSrv_default");
    
    // move_base get plan
    pathReq_client_costmap2D = nh_.serviceClient<robotino_fsm::GetPlan_>(get_plan_costmap2D_topic);
    pathReq_client_gridmap = nh_.serviceClient<robotino_fsm::GetPlan_>(get_plan_gridmap_topic);
    get_best_path = nh_.serviceClient<active_slam::get_best_path>(get_best_path_topic);
    get_best_head = nh_.serviceClient<active_slam::get_best_head>(get_best_head_topic);

    nearby_nodes_client = nh_.serviceClient<rtabmap_ros::GetNearbyNodeData>(get_nearby_nodes_topic);

    clear_costmap = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    clear_obstacles = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/standalone_converter/CostmapToPolygonsDBSMCCH/set_parameters");


    rtabmap_pause = nh_.serviceClient<std_srvs::Empty>("/rtabmap/pause");
    rtabmap_resetodom = nh_.serviceClient<std_srvs::Empty>("/rtabmap/reset_odom");
    rtabmap_newmap = nh_.serviceClient<std_srvs::Empty>("/rtabmap/trigger_new_map");
    rtabmap_resume = nh_.serviceClient<std_srvs::Empty>("/rtabmap/resume");
    ekf_camera_reset = nh_.serviceClient<robot_localization::SetPose>("/camera/set_pose");
    ekf_robot_reset = nh_.serviceClient<robot_localization::SetPose>("/robot/set_pose");


    pubCamCmd = nh_.advertise<geometry_msgs::Twist>("/camera/cmd_vel", 1);
    pubRobCmd = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    spinner.start();

    failed_plans = 0;
    failed_replans = 0;
    replan_cnt = 0;
    failed = 0;

    while (ros::ok()) {
        if (tf_ok) {
            if (require_new_goal) {
                new_goal();
                if (currentPlan.empty() && failed_plans > 10) {
                    emergency = true;
                } else if (!currentPlan.empty()) {
                    ROS_INFO_STREAM("Got new Goal");
                    ROS_INFO("Current Plan Size = %ld", currentPlan.size());

                    emergency = false;
                    is_second = 2;
                }
            }
            // not doing 360, not requiring new goal and need replan
            if (replan && !require_new_goal) {// && !done_360 && !found_lc) {
                replan_goal();
                replan_cnt += 1;
                if (!currentPlan.empty() && send_goal) {
                    is_second = 2;
                }
            }

            if (emergency || replan_cnt > 5 || failed > 5) {
                general_reset();
                ROS_ERROR_STREAM("EMERGENCY RESET!");
            }

            /*if (found_lc) {
                if (counter_lc > 6) {
                    found_lc = false;
                }
            } else {
                found_lc = find_lc();
            }*/

            if (!currentPlan.empty() && send_goal) {
    //                ROS_INFO_STREAM("Sending goal");
                nav_msgs::Odometry goal;
                goal.header.frame_id = "";
                goal.pose.pose = currentPlan.at(0);
                int size = currentPlan.size();
                // ROS_INFO_STREAM("Current Plan Size = %d", size);

                if (currentPlan.size() > 1) {
                    if (currentPlan.size() % 2 == 0 && !done_360) {
                        if (!mid_optimizer) {
                            goal.twist.twist.angular.z = tf::getYaw(currentPlan.at(1).orientation);
                        } else {
                            active_slam::get_best_head srv;

                            srv.request.kind_cost = kind_cost;
                            srv.request.fov_angle_deg = 360;
                            srv.request.pose = currentPlan.at(1);
                            srv.request.length = (double) currentPlan.size() / 2.0;
                            srv.request.theta = tf::getYaw(currentPlan.at(0).orientation) * 180 / M_PI;

                            if (get_best_head.call(srv)) {
                                goal.twist.twist.angular.z = srv.response.orientation;
                                ROS_INFO_STREAM(tf::getYaw(currentPlan.at(1).orientation) * 180 / M_PI -
                                                goal.twist.twist.angular.z * 180 / M_PI);
                                currentPlan.at(1).orientation = tf::createQuaternionMsgFromYaw(
                                        srv.response.orientation);
                            } else {
                                goal.twist.twist.angular.z = tf::getYaw(currentPlan.at(1).orientation);
                            }
                        }
                        goal.twist.twist.linear.x = currentPlan.at(1).position.x;
                        goal.twist.twist.linear.y = currentPlan.at(1).position.y;
                        goal.twist.twist.linear.z = 0;
                    } else {
                        goal.twist.twist.linear.z = -1;
                    }
                } else {
                    goal.twist.twist.linear.z = -1;
                }
                mpcGoalPub.publish(goal);
            }
        }
        ros::Duration(0.2).sleep();
    }
}

void robotino_fsm_node::replan_goal() {
    if (currentPlan.empty()) {
        require_new_goal = true;
        replan = false;
        failed_replans += 1;
        return;
    }

    ros::Duration(1).sleep();

    std::vector<geometry_msgs::PoseStamped> plan;
    switch(world_path_param){
        case 0:
            plan = getPlan_costmap2D(currentPlan.at(currentPlan.size() - 1));
            break;
        case 1:
            plan = getPlan_gridmap_transformation(currentPlan.at(currentPlan.size() - 1));
            break;
        case 2:
            plan = getPlan_gridmap_default(currentPlan.at(currentPlan.size() - 1));
            break;
    }
    double dist = 0;
    if (!plan.empty()) {
        ROS_INFO_STREAM("Replanning plan with new size " << plan.size());
        // get length and waypoints
        std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths;
        std::vector<geometry_msgs::Pose> refinedPlan;
        refinedPlan = refinePlan(plan, dist, 0.5); // get wp 0.5 meter distance
        ROS_INFO_STREAM("Replanning plan refined to  " << refinedPlan.size() << " positions");

        std::vector<geometry_msgs::Pose> replanned;
        if (!refinedPlan.empty()) {// prepare vector
            paths.emplace_back(std::make_pair(refinedPlan, dist));
            replanned.clear();
            // get path based on utility
            replanned = getBestPath(paths, kind_cost);
        }

        if (replanned.empty()) {
            failed_replans += 1;
            ROS_WARN_STREAM("replan failed " << failed_replans);
            if (failed_replans >= 5) {
                // we've problems even with replan, should not reach this [because of failed ...]
                require_new_goal = true;
                filter_last = true;
            } else {
                replan = true;
            }
            send_goal = false;
        } else {
            // we've a plan we can try to go on with that
            currentPlan.clear();
            currentPlan = replanned;
            publishCurrentPlan();

//            failed_replans = 0;
            require_new_goal = false;
            send_goal = true;
            filter_last = false;
            replan = false;
        }
    }
}

bool sortasc(std::pair<std::vector<geometry_msgs::Pose>, double> p1,
             std::pair<std::vector<geometry_msgs::Pose>, double> p2) {
    return p1.second < p2.second;
}

void robotino_fsm_node::new_goal() {
    // new goal
    // get frontier
    std::vector<geometry_msgs::Pose> goals = getFrontierPoints();

    ROS_INFO_STREAM("is goal empty? " << goals.empty());

    std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths;
    std::vector<geometry_msgs::Pose> refinedPlan;

    // NEW : Default Path twins:
    std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> pathsDefault;
    std::vector<geometry_msgs::Pose> refinedPlanDefault;

    std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths_costmap2D;
    std::vector<geometry_msgs::Pose> refinedPlan_costmap2D;

    std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths_gridmap_transformation;
    std::vector<geometry_msgs::Pose> refinedPlan_gridmap_transformation;

    std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths_gridmap_default;
    std::vector<geometry_msgs::Pose> refinedPlan_gridmap_default;
    
    // for each frontier

    if (!goals.empty() && failed_plans <= 5) {
        ROS_INFO_STREAM("WE HAVE SOMETHING");
        if (done_360) {
            currentPlan.clear();
            currentPlan = goals;
            // NEW
            // defaultPlan.clear();
            // defaultPlan = goals;
        } else {
            // for every goal refine + get utility + get plan
            for (auto goal : goals) {
                // get plan
                // std::vector<geometry_msgs::PoseStamped> plan = getPlan(goal);
                // double dist = 0;
                // if (!plan.empty()) {
                //     // get length and waypoints
                //     refinedPlan = refinePlan(plan, dist, 0.5); // get wp 0.5 meter distance
                //     if (!refinedPlan.empty())
                //         // prepare vector
                //         paths.emplace_back(std::make_pair(refinedPlan, dist));
                // }

                // // NEW : Default Path twins:
                // std::vector<geometry_msgs::PoseStamped> default_plan = getDefaultPlan(goal);
                // double default_dist = 0;
                // if (!default_plan.empty()) {
                //     // get length and waypoints
                //     refinedPlanDefault = refinePlan(default_plan, default_dist, 0.5); // get wp 0.5 meter distance
                //     if (!refinedPlanDefault.empty())
                //         // prepare vector
                //         pathsDefault.emplace_back(std::make_pair(refinedPlanDefault, default_dist));
                // }

                //costmap2d
                std::vector<geometry_msgs::PoseStamped> plan_costmap2D = getPlan_costmap2D(goal);
                double dist_costmap2D = 0;
                if (!plan_costmap2D.empty()) {
                    // get length and waypoints
                    refinedPlan_costmap2D = refinePlan(plan_costmap2D, dist_costmap2D, 0.5); // get wp 0.5 meter distance
                    if (!refinedPlan_costmap2D.empty())
                        // prepare vector
                        paths_costmap2D.emplace_back(std::make_pair(refinedPlan_costmap2D, dist_costmap2D));
                }

                //gridmap_transformation
                std::vector<geometry_msgs::PoseStamped> plan_gridmap_transformation = getPlan_gridmap_transformation(goal);
                double dist_gridmap_transformation = 0;
                if (!plan_gridmap_transformation.empty()) {
                    // get length and waypoints
                    refinedPlan_gridmap_transformation = refinePlan(plan_gridmap_transformation, dist_gridmap_transformation, 0.5); // get wp 0.5 meter distance
                    if (!refinedPlan_gridmap_transformation.empty())
                        // prepare vector
                        paths_gridmap_transformation.emplace_back(std::make_pair(refinedPlan_gridmap_transformation, dist_gridmap_transformation));
                }

                //gridmap_default
                std::vector<geometry_msgs::PoseStamped> plan_gridmap_default = getPlan_gridmap_default(goal);
                double dist_gridmap_default = 0;
                if (!plan_gridmap_default.empty()) {
                    // get length and waypoints
                    refinedPlan_gridmap_default = refinePlan(plan_gridmap_default, dist_gridmap_default, 0.5); // get wp 0.5 meter distance
                    if (!refinedPlan_gridmap_default.empty())
                        // prepare vector
                        paths_gridmap_default.emplace_back(std::make_pair(refinedPlan_gridmap_default, dist_gridmap_default));
                }

            }

            // std::sort(paths.begin(), paths.end(), sortasc);
            // for (auto it = paths.begin(); it != paths.end(); it++) {
            //     if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
            //                                  it->first.at(it->first.size() - 1).position.y, 0},
            //              std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) < 0.3) {
            //         paths.erase(it);
            //         it--;
            //         continue;
            //     } else
            //         for (auto it2 = it + 1; it2 != paths.end(); it2++) {
            //             if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
            //                                          it->first.at(it->first.size() - 1).position.y, 0},
            //                      std::vector<double>{it2->first.at(it2->first.size() - 1).position.x,
            //                                          it2->first.at(it2->first.size() - 1).position.y, 0}) < 1) {
            //                 paths.erase(it2--);
            //                 it--;
            //                 break;
            //             }
            //         }
            // }

            // NEW : Default Path twins:
            // std::sort(pathsDefault.begin(), pathsDefault.end(), sortasc);
            // for (auto it = pathsDefault.begin(); it != pathsDefault.end(); it++) {
            //     if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
            //                                  it->first.at(it->first.size() - 1).position.y, 0},
            //              std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) < 0.3) {
            //         pathsDefault.erase(it);
            //         it--;
            //         continue;
            //     } else
            //         for (auto it2 = it + 1; it2 != pathsDefault.end(); it2++) {
            //             if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
            //                                          it->first.at(it->first.size() - 1).position.y, 0},
            //                      std::vector<double>{it2->first.at(it2->first.size() - 1).position.x,
            //                                          it2->first.at(it2->first.size() - 1).position.y, 0}) < 1) {
            //                 pathsDefault.erase(it2--);
            //                 it--;
            //                 break;
            //             }
            //         }
            // }

            // costmap2D
            std::sort(paths_costmap2D.begin(), paths_costmap2D.end(), sortasc);
            for (auto it = paths_costmap2D.begin(); it != paths_costmap2D.end(); it++) {
                if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                             it->first.at(it->first.size() - 1).position.y, 0},
                         std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) < 0.3) {
                    paths_costmap2D.erase(it);
                    it--;
                    continue;
                } else
                    for (auto it2 = it + 1; it2 != paths_costmap2D.end(); it2++) {
                        if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                                     it->first.at(it->first.size() - 1).position.y, 0},
                                 std::vector<double>{it2->first.at(it2->first.size() - 1).position.x,
                                                     it2->first.at(it2->first.size() - 1).position.y, 0}) < 1) {
                            paths_costmap2D.erase(it2--);
                            it--;
                            break;
                        }
                    }
            }

            // gridmap_transformation
            std::sort(paths_gridmap_transformation.begin(), paths_gridmap_transformation.end(), sortasc);
            for (auto it = paths_gridmap_transformation.begin(); it != paths_gridmap_transformation.end(); it++) {
                if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                             it->first.at(it->first.size() - 1).position.y, 0},
                         std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) < 0.3) {
                    paths_gridmap_transformation.erase(it);
                    it--;
                    continue;
                } else
                    for (auto it2 = it + 1; it2 != paths_gridmap_transformation.end(); it2++) {
                        if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                                     it->first.at(it->first.size() - 1).position.y, 0},
                                 std::vector<double>{it2->first.at(it2->first.size() - 1).position.x,
                                                     it2->first.at(it2->first.size() - 1).position.y, 0}) < 1) {
                            paths_gridmap_transformation.erase(it2--);
                            it--;
                            break;
                        }
                    }
            }

            // gridmap_default
            std::sort(paths_gridmap_default.begin(), paths_gridmap_default.end(), sortasc);
            for (auto it = paths_gridmap_default.begin(); it != paths_gridmap_default.end(); it++) {
                if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                             it->first.at(it->first.size() - 1).position.y, 0},
                         std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) < 0.3) {
                    paths_gridmap_default.erase(it);
                    it--;
                    continue;
                } else
                    for (auto it2 = it + 1; it2 != paths_gridmap_default.end(); it2++) {
                        if (dist(std::vector<double>{it->first.at(it->first.size() - 1).position.x,
                                                     it->first.at(it->first.size() - 1).position.y, 0},
                                 std::vector<double>{it2->first.at(it2->first.size() - 1).position.x,
                                                     it2->first.at(it2->first.size() - 1).position.y, 0}) < 1) {
                            paths_gridmap_default.erase(it2--);
                            it--;
                            break;
                        }
                    }
            }

            // while (paths.size() > 4) {
            //     paths.erase(paths.end());
            // }

            // // NEW : Default Path twins:
            // while (pathsDefault.size() > 4) {
            //     pathsDefault.erase(pathsDefault.end());
            // }

            // NEW : Default Path twins:
            while (paths_costmap2D.size() > 4) {
                paths_costmap2D.erase(paths_costmap2D.end());
            }

            while (paths_gridmap_transformation.size() > 4) {
                paths_gridmap_transformation.erase(paths_gridmap_transformation.end());
            }

            while (paths_gridmap_default.size() > 4) {
                paths_gridmap_default.erase(paths_gridmap_default.end());
            }

            // get path based on utility
            // currentPlan = getBestPath(paths, kind_cost);
            // LogCurrentStats();
            // defaultPlan = getBestPath(pathsDefault, kind_cost);
            // LogDefaultStats();

            costmap2D_Plan = getBestPath(paths_costmap2D, kind_cost);
            LogCostmap2DStats();
            gridmap_transformation_Plan = getBestPath(paths_gridmap_transformation, kind_cost);
            LogGridmapTransformationStats();
            gridmap_default_Plan = getBestPath(paths_gridmap_default, kind_cost);
            LogGridmapDefaultStats();

        }
    } else {
        if (failed_plans > 20) {
            failed_plans = 0;
        }
        std::vector<geometry_msgs::PoseStamped> plan = revisit_goal();
        // std::vector<geometry_msgs::PoseStamped> default_plan = plan;
        double dist = 0;
        // double default_dist = 0;

        if (!plan.empty()) {
            // get length and waypoints
            refinedPlan = refinePlan(plan, dist, 0.5); // get wp 0.5 meter distance
            if (!refinedPlan.empty()) {
                // prepare vector
                paths.emplace_back(std::make_pair(refinedPlan, dist));
            } else {
                currentPlan.clear();
            }
            if (!paths.empty()) {
                currentPlan.clear();
                currentPlan = getBestPath(paths, kind_cost);
                // LogCurrentStats();

            }                // get path based on utility
        }

        // if (!default_plan.empty()) {
        //     // get length and waypoints
        //     refinedPlanDefault = refinePlan(default_plan, default_dist, 0.5); // get wp 0.5 meter distance
        //     if (!refinedPlanDefault.empty()) {
        //         // prepare vector
        //         pathsDefault.emplace_back(std::make_pair(refinedPlanDefault, default_dist));
        //     } else {
        //         defaultPlan.clear();
        //     }
        //     if (!pathsDefault.empty()) {
        //         defaultPlan.clear();
        //         defaultPlan = getBestPath(pathsDefault, kind_cost);
        //         LogDefaultStats();
        //     }                // get path based on utility
        // }
    }

    // at this point 99.99% of the times we have a goal in currentPlan

    // std::cout<<"WORLD_PATH_PARAM: "<<world_path_param<<std::endl;
    C2D_plan = costmap2D_Plan;
    GM_plan = gridmap_default_Plan;
    TRN_plan = gridmap_transformation_Plan;

    switch (world_path_param)
    {
    case 0:
        currentPlan = costmap2D_Plan;
        std::cout<<"Chosen costmap2D Plan"<<std::endl;
        break;
    case 1:
        currentPlan = gridmap_transformation_Plan;
        std::cout<<"Chosen gridmap_transformation Plan"<<std::endl;
        break;
    case 2:
        currentPlan = gridmap_default_Plan;
        std::cout<<"Chosen gridmap_default Plan"<<std::endl;
        break;
    }

    if (currentPlan.empty()) {
        failed_plans += 1;
        require_new_goal = true;
        send_goal = false;
        if (failed_plans > 5 && !goals.empty()) {
            currentPlan.push_back(goals.at(goals.size() - 1));
        }
        ROS_WARN_STREAM("Empty current plan, retry. Failed " << failed_plans << " plans.");
    } else {
        ROS_INFO_STREAM("Best path identified toward");
        ROS_INFO_STREAM(currentPlan.at(currentPlan.size() - 1).position);
        failed_plans = 0;
        require_new_goal = false;
        send_goal = true;
        replan = false;
         //NEW ADDITION
        publishCurrentPlan();
        publishC2D_Plan();
        publishGM_Plan();
        publishTRN_Plan();
        // publishDefaultPlan();
        publishCurrentPaths(paths_gridmap_transformation);
        publishDefaultPaths(paths_costmap2D);
        publishGridmapPaths(paths_gridmap_default);
        std::cout<<"\n";
        ros::Time timeNow = ros::Time::now();
        gp_current.stamp = timeNow;
        gp_default.stamp = timeNow;
        gp_costmap2D.stamp = timeNow;
        gp_gridmap_transformation.stamp = timeNow;
        gp_gridmap_default.stamp = timeNow;

        // pubGroupedCurrentPath.publish(gp_current);
        // pubGroupedDefaultPath.publish(gp_default);
        pubGroupedCostmap2DPath.publish(gp_costmap2D);
        pubGroupedGridmapTransformationPath.publish(gp_gridmap_transformation);
        pubGroupedGridmapDefaultPath.publish(gp_gridmap_default);
    }

}

// NEW ADDITION

void robotino_fsm_node::publishCurrentPaths(std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths)
{
    std::cout<<"\nPublishing Current Paths.\n";
    std::cout<<"Current Path Set Size: "<<paths.size()<<"\n";

    nav_msgs::Path emptyPath;
    emptyPath.header.frame_id = "map";
    emptyPath.poses.resize(1);
    emptyPath.poses[0].pose.position.x = 0.0;
    emptyPath.poses[0].pose.position.y = 0.0;
    emptyPath.poses[0].pose.position.z = 0.0;

    int pass = 0;
    for(auto path_pairs : paths)
    {       
            nav_msgs::Path path;
            path.header.frame_id = "map";
            path.poses.resize(path_pairs.first.size());

            for(int i = 0; i < path_pairs.first.size(); i++)
            {
                path.poses[i].pose = path_pairs.first.at(i);
            }

            pubCurrentPaths.publish(path);
            pass++;
    }
    while(pass < 4)
    {
        pubCurrentPaths.publish(emptyPath);
        pass++;
    }
}

void robotino_fsm_node::publishDefaultPaths(std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths)
{
    std::cout<<"\nPublishing Default Paths.\n";
    std::cout<<"Default Path Set Size: "<<paths.size()<<"\n";

    nav_msgs::Path emptyPath;
    emptyPath.header.frame_id = "map";
    emptyPath.poses.resize(1);
    emptyPath.poses[0].pose.position.x = 0.0;
    emptyPath.poses[0].pose.position.y = 0.0;
    emptyPath.poses[0].pose.position.z = 0.0;
    
    int pass = 0;
    for(auto path_pairs : paths)
    {       
            nav_msgs::Path path;
            path.header.frame_id = "map";
            path.poses.resize(path_pairs.first.size());

            for(int i = 0; i < path_pairs.first.size(); i++)
            {
                path.poses[i].pose = path_pairs.first.at(i);
            }

            pubDefaultPaths.publish(path);
            pass++;
    }
    while(pass < 4)
    {
        pubDefaultPaths.publish(emptyPath);
        pass++;
    }
}

void robotino_fsm_node::publishGridmapPaths(std::vector<std::pair<std::vector<geometry_msgs::Pose>, double >> paths)
{
    std::cout<<"\nPublishing Gridmap Paths.\n";
    std::cout<<"Gridmap Path Set Size: "<<paths.size()<<"\n";

    nav_msgs::Path emptyPath;
    emptyPath.header.frame_id = "map";
    emptyPath.poses.resize(1);
    emptyPath.poses[0].pose.position.x = 0.0;
    emptyPath.poses[0].pose.position.y = 0.0;
    emptyPath.poses[0].pose.position.z = 0.0;
    
    int pass = 0;
    for(auto path_pairs : paths)
    {       
            nav_msgs::Path path;
            path.header.frame_id = "map";
            path.poses.resize(path_pairs.first.size());

            for(int i = 0; i < path_pairs.first.size(); i++)
            {
                path.poses[i].pose = path_pairs.first.at(i);
            }

            pubGridmapPaths.publish(path);
            pass++;
    }
    while(pass < 4)
    {
        pubGridmapPaths.publish(emptyPath);
        pass++;
    }
}

void robotino_fsm_node::publishCurrentPlan()
{
    nav_msgs::Path current_plan_path;

    current_plan_path.header.frame_id = "map";
    current_plan_path.poses.resize(currentPlan.size());

    for(int i = 0; i < currentPlan.size(); i++)
    {
        current_plan_path.poses[i].pose = currentPlan.at(i);
    }

    pubCurrentPlanPath.publish(current_plan_path);
    ROS_INFO_STREAM("Publishing Current Best Path.");    

}

void robotino_fsm_node::publishC2D_Plan()
{
    nav_msgs::Path current_plan_path;

    current_plan_path.header.frame_id = "map";
    current_plan_path.poses.resize(C2D_plan.size());

    for(int i = 0; i < C2D_plan.size(); i++)
    {
        current_plan_path.poses[i].pose = C2D_plan.at(i);
    }

    pubC2DPlanPath.publish(current_plan_path);
    ROS_INFO_STREAM("Publishing C2D Best Path.");    

}

void robotino_fsm_node::publishGM_Plan()
{
    nav_msgs::Path current_plan_path;

    current_plan_path.header.frame_id = "map";
    current_plan_path.poses.resize(GM_plan.size());

    for(int i = 0; i < GM_plan.size(); i++)
    {
        current_plan_path.poses[i].pose = GM_plan.at(i);
    }

    pubGMPlanPath.publish(current_plan_path);
    ROS_INFO_STREAM("Publishing GM Best Path.");    

}

void robotino_fsm_node::publishTRN_Plan()
{
    nav_msgs::Path current_plan_path;

    current_plan_path.header.frame_id = "map";
    current_plan_path.poses.resize(TRN_plan.size());

    for(int i = 0; i < TRN_plan.size(); i++)
    {
        current_plan_path.poses[i].pose = TRN_plan.at(i);
    }

    pubTRNPlanPath.publish(current_plan_path);
    ROS_INFO_STREAM("Publishing TRN Best Path.");    

}

// NEW ADDITION 
void robotino_fsm_node::publishDefaultPlan()
{
    nav_msgs::Path default_plan_path;

    default_plan_path.header.frame_id = "map";
    default_plan_path.poses.resize(defaultPlan.size());

    for(int i = 0; i < defaultPlan.size(); i++)
    {
        default_plan_path.poses[i].pose = defaultPlan.at(i);
    }

    pubDefaultCurrentPlanPath.publish(default_plan_path);
    ROS_INFO_STREAM("Publishing Default Best Path.");

}

void robotino_fsm_node::LogCurrentStats(){

    ROS_INFO_STREAM("LOGGIN: Current Path.");
    gp_current.grouped_paths.clear();
    gp_current.grouped_paths.resize(groupInfo.size());
    gp_current.no_of_paths = groupInfo.size();
    for(int i = 0; i< groupInfo.size(); i++){
        gp_current.grouped_paths[i].length = groupInfo[i].length;
        gp_current.grouped_paths[i].entropy_free = groupInfo[i].entropy_free;
        gp_current.grouped_paths[i].entropy = groupInfo[i].entropy;
        gp_current.grouped_paths[i].x = groupInfo[i].x;
        gp_current.grouped_paths[i].y = groupInfo[i].y;
        gp_current.grouped_paths[i].no_of_waypoints = groupInfo[i].no_of_waypoints;

    }
    gp_current.best_path.entropy = bestInfo.entropy;
    gp_current.best_path.entropy_free = bestInfo.entropy_free;
    gp_current.best_path.length = bestInfo.length;
    gp_current.best_path.x = bestInfo.x;
    gp_current.best_path.y = bestInfo.y;
    gp_current.best_path.no_of_waypoints = bestInfo.no_of_waypoints;

}

void robotino_fsm_node::LogDefaultStats(){

    ROS_INFO_STREAM("LOGGIN: Default Path.");

    gp_default.grouped_paths.clear();
    gp_default.grouped_paths.resize(groupInfo.size());
    gp_default.no_of_paths = groupInfo.size();
    for(int i = 0; i< groupInfo.size(); i++){
        gp_default.grouped_paths[i].length = groupInfo[i].length;
        gp_default.grouped_paths[i].entropy_free = groupInfo[i].entropy_free;
        gp_default.grouped_paths[i].entropy = groupInfo[i].entropy;
        gp_default.grouped_paths[i].x = groupInfo[i].x;
        gp_default.grouped_paths[i].y = groupInfo[i].y;
        gp_default.grouped_paths[i].no_of_waypoints = groupInfo[i].no_of_waypoints;

    }
    gp_default.best_path.entropy = bestInfo.entropy;
    gp_default.best_path.entropy_free = bestInfo.entropy_free;
    gp_default.best_path.length = bestInfo.length;
    gp_default.best_path.x = bestInfo.x;
    gp_default.best_path.y = bestInfo.y;
    gp_default.best_path.no_of_waypoints = bestInfo.no_of_waypoints;

}

void robotino_fsm_node::LogCostmap2DStats(){

    ROS_INFO_STREAM("LOGGIN: Costmap2DStats.");
    gp_costmap2D.grouped_paths.clear();
    gp_costmap2D.grouped_paths.resize(groupInfo.size());
    gp_costmap2D.no_of_paths = groupInfo.size();
    for(int i = 0; i< groupInfo.size(); i++){
        gp_costmap2D.grouped_paths[i].length = groupInfo[i].length;
        gp_costmap2D.grouped_paths[i].entropy_free = groupInfo[i].entropy_free;
        gp_costmap2D.grouped_paths[i].entropy = groupInfo[i].entropy;
        gp_costmap2D.grouped_paths[i].x = groupInfo[i].x;
        gp_costmap2D.grouped_paths[i].y = groupInfo[i].y;
        gp_costmap2D.grouped_paths[i].no_of_waypoints = groupInfo[i].no_of_waypoints;

    }
    gp_costmap2D.best_path.entropy = bestInfo.entropy;
    gp_costmap2D.best_path.entropy_free = bestInfo.entropy_free;
    gp_costmap2D.best_path.length = bestInfo.length;
    gp_costmap2D.best_path.x = bestInfo.x;
    gp_costmap2D.best_path.y = bestInfo.y;
    gp_costmap2D.best_path.no_of_waypoints = bestInfo.no_of_waypoints;

}

void robotino_fsm_node::LogGridmapTransformationStats(){

    ROS_INFO_STREAM("LOGGIN: GridmapTransformationStats.");
    gp_gridmap_transformation.grouped_paths.clear();
    gp_gridmap_transformation.grouped_paths.resize(groupInfo.size());
    gp_gridmap_transformation.no_of_paths = groupInfo.size();
    for(int i = 0; i< groupInfo.size(); i++){
        gp_gridmap_transformation.grouped_paths[i].length = groupInfo[i].length;
        gp_gridmap_transformation.grouped_paths[i].entropy_free = groupInfo[i].entropy_free;
        gp_gridmap_transformation.grouped_paths[i].entropy = groupInfo[i].entropy;
        gp_gridmap_transformation.grouped_paths[i].x = groupInfo[i].x;
        gp_gridmap_transformation.grouped_paths[i].y = groupInfo[i].y;
        gp_gridmap_transformation.grouped_paths[i].no_of_waypoints = groupInfo[i].no_of_waypoints;

    }
    gp_gridmap_transformation.best_path.entropy = bestInfo.entropy;
    gp_gridmap_transformation.best_path.entropy_free = bestInfo.entropy_free;
    gp_gridmap_transformation.best_path.length = bestInfo.length;
    gp_gridmap_transformation.best_path.x = bestInfo.x;
    gp_gridmap_transformation.best_path.y = bestInfo.y;
    gp_gridmap_transformation.best_path.no_of_waypoints = bestInfo.no_of_waypoints;

}

void robotino_fsm_node::LogGridmapDefaultStats(){

    ROS_INFO_STREAM("LOGGIN: GridmapDefaultStats.");
    gp_gridmap_default.grouped_paths.clear();
    gp_gridmap_default.grouped_paths.resize(groupInfo.size());
    gp_gridmap_default.no_of_paths = groupInfo.size();
    for(int i = 0; i< groupInfo.size(); i++){
        gp_gridmap_default.grouped_paths[i].length = groupInfo[i].length;
        gp_gridmap_default.grouped_paths[i].entropy_free = groupInfo[i].entropy_free;
        gp_gridmap_default.grouped_paths[i].entropy = groupInfo[i].entropy;
        gp_gridmap_default.grouped_paths[i].x = groupInfo[i].x;
        gp_gridmap_default.grouped_paths[i].y = groupInfo[i].y;
        gp_gridmap_default.grouped_paths[i].no_of_waypoints = groupInfo[i].no_of_waypoints;

    }
    gp_gridmap_default.best_path.entropy = bestInfo.entropy;
    gp_gridmap_default.best_path.entropy_free = bestInfo.entropy_free;
    gp_gridmap_default.best_path.length = bestInfo.length;
    gp_gridmap_default.best_path.x = bestInfo.x;
    gp_gridmap_default.best_path.y = bestInfo.y;
    gp_gridmap_default.best_path.no_of_waypoints = bestInfo.no_of_waypoints;

}

std::vector<geometry_msgs::Pose>
robotino_fsm_node::getBestPath(const std::vector<std::pair<std::vector<geometry_msgs::Pose>, double>> &paths, int kind,
                               int fov) {
    active_slam::get_best_path srv;
    active_slam::SinglePath sp;

    groupInfo.clear();
    groupInfo.resize(paths.size());

    for (const auto &i   : paths) {
        sp.waypoints = i.first;
        sp.length = i.second;
        srv.request.paths.data.push_back(sp);
    }
    srv.request.kind_cost = kind;
    srv.request.fov_angle_deg = 180;
    srv.request.only_last = only_last_set;
    ROS_INFO_STREAM("get best path among " << paths.size());
    if (get_best_path.call(srv)) {
        if (only_last_set && pre_fix) {
            ROS_INFO_STREAM("Pre-fixing");
            // fix after chosing based on last
            srv.request.kind_cost = kind;
            srv.request.only_last = false;
            srv.request.paths.data.clear();

            sp.waypoints = srv.response.path.waypoints;
            sp.length = srv.response.path.length;
            srv.request.paths.data.push_back(sp);

            if (get_best_path.call(srv)) {
entropy_ = srv.response.entropy_path_;
                length_ = srv.response.path.length;

                for(int i = 0; i < paths.size(); i++){
                    groupInfo[i].entropy = srv.response.grouped_paths[i].entropy;
                    groupInfo[i].entropy_free = srv.response.grouped_paths[i].entropy_free;
                    groupInfo[i].length = srv.response.grouped_paths[i].length;
                    groupInfo[i].x = srv.response.grouped_paths[i].x;
                    groupInfo[i].y = srv.response.grouped_paths[i].y;
                    groupInfo[i].no_of_waypoints = srv.response.grouped_paths[i].no_of_waypoints;
                }
                bestInfo.entropy = srv.response.best_path.entropy;
                bestInfo.entropy_free = srv.response.best_path.entropy_free;
                bestInfo.length = srv.response.best_path.length;
                bestInfo.x = srv.response.best_path.x;
                bestInfo.y = srv.response.best_path.y;
                bestInfo.no_of_waypoints = srv.response.best_path.no_of_waypoints;

                return srv.response.path.waypoints;
            }
        }
        entropy_ = srv.response.entropy_path_;
        length_ = srv.response.path.length;

        for(int i = 0; i < paths.size(); i++){
            groupInfo[i].entropy = srv.response.grouped_paths[i].entropy;
            groupInfo[i].entropy_free = srv.response.grouped_paths[i].entropy_free;
            groupInfo[i].length = srv.response.grouped_paths[i].length;
            groupInfo[i].x = srv.response.grouped_paths[i].x;
            groupInfo[i].y = srv.response.grouped_paths[i].y;
            groupInfo[i].no_of_waypoints = srv.response.grouped_paths[i].no_of_waypoints;
        }
        bestInfo.entropy = srv.response.best_path.entropy;
        bestInfo.entropy_free = srv.response.best_path.entropy_free;
        bestInfo.length = srv.response.best_path.length;
        bestInfo.x = srv.response.best_path.x;
        bestInfo.y = srv.response.best_path.y;
        bestInfo.no_of_waypoints = srv.response.best_path.no_of_waypoints;

        return srv.response.path.waypoints;
    }
    return std::vector<geometry_msgs::Pose>{};
}

std::vector<geometry_msgs::Pose> robotino_fsm_node::getFrontierPoints() {
    active_slam::get_frontier_list srv;
    std::vector<geometry_msgs::Pose> goals;
    if (get_frontier.call(srv)) {
        geometry_msgs::Pose tmp;
        for (auto i : srv.response.frontier_list) {
            tmp.position.x = i.x;
            tmp.position.y = i.y;
            if (dist(std::vector<double>{tmp.position.x, tmp.position.y, tmp.position.z},
                     std::vector<double>{globalRobot.position.x, globalRobot.position.y, 0}) > 1) {
                if (!currentPlan.empty()) {
                    if (dist(std::vector<double>{tmp.position.x, tmp.position.y, 0},
                             std::vector<double>{currentPlan.at(currentPlan.size() - 1).position.x,
                                                 currentPlan.at(currentPlan.size() - 1).position.y,
                                                 0}) >
                        (filter_last ? 1 : 0.5))
                        goals.push_back(tmp);
                } else {
                    goals.push_back(tmp);
                }
            }
        }
        ROS_INFO_STREAM("Received " << goals.size() << " goals.");
        if (!goals.empty()) { done_360 = false; }
    }

    if (goals.empty()) {
        if (!done_360 && !filter_last) {
            goals.clear();
            currentPlan.clear();
            done_360 = true;
            geometry_msgs::Pose tmp;
            tmp.position.x = globalRobot.position.x;
            tmp.position.y = globalRobot.position.y;

            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw_rob + yaw_cam);
            goals.push_back(tmp);

            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw_rob + yaw_cam + M_PI / 2);
            goals.push_back(tmp);

            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw_rob + yaw_cam + M_PI);
            goals.push_back(tmp);

            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw_rob + yaw_cam + 1.5 * M_PI);
            goals.push_back(tmp);

            tmp.orientation = tf::createQuaternionMsgFromYaw(yaw_rob + yaw_cam + 2 * M_PI);
            goals.push_back(tmp);
            ROS_WARN_STREAM("Recovery 360.");
        } else {
            ROS_WARN_STREAM("No goals available for some reasons. Check.");
            return std::vector<geometry_msgs::Pose>{};
        }
    }
    return goals;
}

bool isGlobalLoop(int x) {
    return x == rtabmap::Link::kGlobalClosure || x == rtabmap::Link::kNeighborMerged ||
           x == rtabmap::Link::kLocalSpaceClosure;
}

bool isLocalLoop(int x) {
    return x == rtabmap::Link::kLocalSpaceClosure;
}

std::vector<geometry_msgs::PoseStamped> robotino_fsm_node::revisit_goal() {
    rtabmap_ros::GetNearbyNodeData srvRtab;
    srvRtab.request.pose.pose.position = globalRobot.position;
    srvRtab.request.pose.pose.orientation = globalRobot.orientation; // not used
    srvRtab.request.rad = 10;
    srvRtab.request.k = 1000;
    if (nearby_nodes_client.call(srvRtab)) {
        auto iter = srvRtab.response.value.begin();
        int dist = -1;
        geometry_msgs::PosePtr start = geometry_msgs::PosePtr(new geometry_msgs::Pose);

        *start = globalRobot;

        std::vector<geometry_msgs::PoseStamped> path_toward;
        if (!srvRtab.response.data.empty()) {
//            iter = std::find_if(iter, srvRtab.response.value.end(), isGlobalLoop);
//            while (iter != srvRtab.response.value.end()) {
//
//                path_toward = getPlan(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).pose, start);
//                int curr_dist = (int) (path_toward.size());
//
//                if (curr_dist > dist && curr_dist != 0) {
//                    dist = curr_dist;
//
//                    tf::Quaternion a;
//                    tf::quaternionMsgToTF(path_toward.at(path_toward.size() - 1).pose.orientation, a);
//                    tf::Quaternion b;
//                    b.setX(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
//                            0).rotation.x);
//                    b.setY(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
//                            0).rotation.y);
//                    b.setZ(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
//                            0).rotation.z);
//                    b.setW(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
//                            0).rotation.w);
//                    b = tf::createQuaternionFromYaw(tf::getYaw(b));
//                    tf::quaternionTFToMsg(a * b, path_toward.at(path_toward.size() - 1).pose.orientation);
//
//                }
//                iter = std::find_if(iter + 1, srvRtab.response.value.end(), isGlobalLoop);
//            }
            if (dist > 0)
                return path_toward;
            else {
                int r = rand() % srvRtab.response.data.size();

                int cnt = 0;
                while (srvRtab.response.data.at(r).wordPts.size() < 400 && cnt < srvRtab.response.data.size()) {
                    r = rand() % srvRtab.response.data.size();
                    cnt += 1;
                }

                path_toward = getPlan(srvRtab.response.data.at(r).pose, start);
                if (!path_toward.empty()) {
                    tf::Quaternion a;
                    tf::quaternionMsgToTF(path_toward.at(path_toward.size() - 1).pose.orientation, a);
                    tf::Quaternion b;
                    b.setX(srvRtab.response.data.at(r).localTransform.at(0).rotation.x);
                    b.setY(srvRtab.response.data.at(r).localTransform.at(0).rotation.y);
                    b.setZ(srvRtab.response.data.at(r).localTransform.at(0).rotation.z);
                    b.setW(srvRtab.response.data.at(r).localTransform.at(0).rotation.w);
                    b = tf::createQuaternionFromYaw(tf::getYaw(b));
                    tf::quaternionTFToMsg(a * b, path_toward.at(path_toward.size() - 1).pose.orientation);
                    return path_toward;
                } else return std::vector<geometry_msgs::PoseStamped>{};
            }
        } else return std::vector<geometry_msgs::PoseStamped>{};
    } else { return std::vector<geometry_msgs::PoseStamped>{}; }
}

bool robotino_fsm_node::find_lc() {
    rtabmap_ros::GetNearbyNodeData srvRtab;
    srvRtab.request.pose.pose.position = globalRobot.position;
    srvRtab.request.pose.pose.orientation = globalRobot.orientation; // not used
    srvRtab.request.rad = 1;
    srvRtab.request.k = 100;
    if (nearby_nodes_client.call(srvRtab)) {
        auto iter = srvRtab.response.value.begin();
        int dist = -1;
        geometry_msgs::PosePtr start = geometry_msgs::PosePtr(new geometry_msgs::Pose);

        *start = globalRobot;

        geometry_msgs::PosePtr end = geometry_msgs::PosePtr(new geometry_msgs::Pose);

        *end = currentPlan.size() > 1 ? currentPlan.at(1) : globalRobot;

        std::vector<geometry_msgs::PoseStamped> path_toward;
        std::vector<geometry_msgs::PoseStamped> path_backward;

        iter = std::find_if(iter, srvRtab.response.value.end(), isGlobalLoop);

        while (iter != srvRtab.response.value.end()) {
            path_toward = getPlan(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).pose, start);
            path_backward = getPlan(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).pose, end);
            int curr_dist = (int) (path_toward.size() + path_backward.size());

            if (dist == -1) {
                if (curr_dist != 0) {
                    dist = curr_dist;
                    tf::Quaternion a;
                    tf::quaternionMsgToTF(path_toward.at(path_toward.size() - 1).pose.orientation, a);
                    tf::Quaternion b;
                    b.setX(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.x);
                    b.setY(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.y);
                    b.setZ(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.z);
                    b.setW(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.w);
                    b = tf::createQuaternionFromYaw(tf::getYaw(b));
                    tf::quaternionTFToMsg(a * b, path_toward.at(path_toward.size() - 1).pose.orientation);
                }
            } else {
                if (curr_dist < dist && curr_dist != 0) {
                    dist = curr_dist;

                    tf::Quaternion a;
                    tf::quaternionMsgToTF(path_toward.at(path_toward.size() - 1).pose.orientation, a);
                    tf::Quaternion b;
                    b.setX(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.x);
                    b.setY(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.y);
                    b.setZ(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.z);
                    b.setW(srvRtab.response.data.at(iter - srvRtab.response.value.begin()).localTransform.at(
                            0).rotation.w);
                    b = tf::createQuaternionFromYaw(tf::getYaw(b));
                    tf::quaternionTFToMsg(a * b, path_toward.at(path_toward.size() - 1).pose.orientation);
                }
            }
            iter = std::find_if(iter + 1, srvRtab.response.value.end(), isGlobalLoop);
        }

        if (dist > 0) { //found
            if (currentPlan.size() > 1)
                currentPlan.erase(currentPlan.begin()); // remove current

            if (currentPlan.empty()) {
                geometry_msgs::Pose current;
                current.position = globalRobot.position;
                current.orientation = tf::createQuaternionMsgFromYaw((yaw_rob + yaw_cam));
                currentPlan.push_back(current);
            }

            // add halfway toward lc, lc, and halfway toward next goal
            geometry_msgs::Pose half_lc;
            half_lc.position = path_toward.at((int) path_toward.size() / 2).pose.position;
            geometry_msgs::Pose lc;
            lc = path_toward.at((int) path_toward.size() - 1).pose;
            half_lc.orientation = tf::createQuaternionMsgFromYaw(
                    (yaw_rob + yaw_cam + tf::getYaw(lc.orientation)) / 2);
            geometry_msgs::Pose half_next;
            half_next = path_backward.at((int) path_backward.size() / 2).pose;
            half_next.orientation = tf::createQuaternionMsgFromYaw(
                    (tf::getYaw(currentPlan.at(0).orientation) + tf::getYaw(lc.orientation)) / 2);
            currentPlan.emplace(currentPlan.begin(), half_next);
            currentPlan.emplace(currentPlan.begin(), lc);
            currentPlan.emplace(currentPlan.begin(), half_lc);
            send_goal = true;
            counter_lc = 0;
            return true;
        }
    }
    return false;
}


std::vector<geometry_msgs::Pose>
robotino_fsm_node::refinePlan(std::vector<geometry_msgs::PoseStamped> plan, double &tot_dist, double thr) {
    std::vector<geometry_msgs::Pose> wp;

    std::vector<double> start;
    start.resize(3);
    start[0] = plan.at(0).pose.position.x;
    start[1] = plan.at(0).pose.position.y;
    start[2] = 0;

    wp.emplace_back(plan.at(0).pose); // always starting position included
    std::vector<double> end;
    end.resize(3);
    end[2] = 0;

    double cum_dist = 0;
    for (int i = 1; i < plan.size(); i++) {
        end[0] = plan.at(i).pose.position.x;
        end[1] = plan.at(i).pose.position.y;
        cum_dist += dist(start, end);
        tot_dist += dist(start, end);
        start = end;
        if (cum_dist >= thr) {
            wp.push_back(plan.at(i).pose);
            cum_dist = 0;
        }
    }
    bool popped = false;
    geometry_msgs::Pose pose_saved;
    if (cum_dist <= thr / 2) {
        pose_saved = wp.at(wp.size() - 1);
        wp.pop_back();
    }
    // ROS_INFO_STREAM("The refined plan has " << wp.size() << " poses");
    if (wp.empty() || wp.size() == 1) {
        return std::vector<geometry_msgs::Pose>{};
    }
    // we want odd size so we can have utility every meter starting from current pose.
    if (wp.size() % 2 == 0) {
        if (popped)
            wp.push_back(wp.at(wp.size() - 1));
        else if (wp.size() > 2)
            wp.pop_back();
        else {
            pose_saved.position.x = (pose_saved.position.x + wp.at(wp.size() - 1).position.x) / 2;
            pose_saved.position.y = (pose_saved.position.y + wp.at(wp.size() - 1).position.y) / 2;
            wp.push_back(pose_saved);
        }
    }

    // ROS_INFO_STREAM("The refined plan has " << wp.size() << " final poses (odd)");
    return wp;
}

std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getPlan(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;
    //NEW ADDITION
    getPlan.request.costs_grid.clear();
    for(int i = 0; i< 1000*1000; i++){
        getPlan.request.costs_grid.push_back(costs_grid[i]);
    }
    std::cout<<"\ngetPlan called!\n";


    if (start != nullptr) {
        getPlan.request.start.pose = *start;
        //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
    } else {
        getPlan.request.start.pose = globalRobot;
        getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
    }
    //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

    int cnt = 0;
    while (cnt < 5) {
        if (pathReq_client.call(getPlan)) {
            //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
            return getPlan.response.plan.poses;
        } else {
            cnt += 1;
        }
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}

std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getPlan_costmap2D(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;
    //NEW ADDITION
    robotino_fsm::getCostmap_ getCostmap_srv;
    getCostmap_srv.request.getcostmap = true;
    if(getCostmapClientTransformation.call(getCostmap_srv))
    // if(true)
    {
        getPlan.request.costs_grid = getCostmap_srv.response.costarray;

        // std::cout<<"\ngetPlan called!"<<std::endl;
        std::cout<<"costmap2D costmap received!"<<std::endl;

        if (start != nullptr) {
            getPlan.request.start.pose = *start;
            //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
        } else {
            getPlan.request.start.pose = globalRobot;
            getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
        }
        //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

        int cnt = 0;
        while (cnt < 5) {
            if (pathReq_client_costmap2D.call(getPlan)) {
                //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
                return getPlan.response.plan.poses;
            } else {
                cnt += 1;
                std::cout<<"WRONTWOTNTASD"<<std::endl;
            }
        }
    }
    else{
        std::cout<<"No Costmap Received\n"<<std::endl;
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}

std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getPlan_gridmap_transformation(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;

    robotino_fsm::getCostmap_ getCostmap_srv;
    getCostmap_srv.request.getcostmap = true;
    if(getCostmapClientTransformation.call(getCostmap_srv))
    {
        getPlan.request.costs_grid = getCostmap_srv.response.costarray;

        // std::cout<<"\ngetPlan called!"<<std::endl;
        std::cout<<"gridmap transformation costmap received!"<<std::endl;

        if (start != nullptr) {
            getPlan.request.start.pose = *start;
            //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
        } else {
            getPlan.request.start.pose = globalRobot;
            getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
        }
        //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

        int cnt = 0;
        while (cnt < 5) {
            if (pathReq_client_gridmap.call(getPlan)) {
                //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
                return getPlan.response.plan.poses;
            } else {
                cnt += 1;
                std::cout<<"WRONTWOTNTASD_gridmap"<<std::endl;
            }
        }
    }
    else{
        std::cout<<"No Costmap Received\n"<<std::endl;
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}

std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getPlan_gridmap_default(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;

    robotino_fsm::getCostmap_ getCostmap_srv;
    getCostmap_srv.request.getcostmap = true;
    if(getCostmapClientDefault.call(getCostmap_srv))
    {
        getPlan.request.costs_grid = getCostmap_srv.response.costarray;

        std::cout<<"\ngetPlan called!\n";

        if (start != nullptr) {
            getPlan.request.start.pose = *start;
            //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
        } else {
            getPlan.request.start.pose = globalRobot;
            getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
        }
        //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

        int cnt = 0;
        while (cnt < 5) {
            if (pathReq_client_gridmap.call(getPlan)) {
                //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
                return getPlan.response.plan.poses;
            } else {
                cnt += 1;
            }
        }
    }
    else{
        std::cout<<"No Costmap Received\n"<<std::endl;
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}

std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getPlan_gridmap(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;

    robotino_fsm::getCostmap_ getCostmap_srv;
    getCostmap_srv.request.getcostmap = true;
    if(getCostmapClientDefault.call(getCostmap_srv))
    {
        getPlan.request.costs_grid = getCostmap_srv.response.costarray;

        std::cout<<"\ngetPlan called!\n";

        if (start != nullptr) {
            getPlan.request.start.pose = *start;
            //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
        } else {
            getPlan.request.start.pose = globalRobot;
            getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
        }
        //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

        int cnt = 0;
        while (cnt < 5) {
            if (pathReq_client_gridmap.call(getPlan)) {
                //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
                return getPlan.response.plan.poses;
            } else {
                cnt += 1;
            }
        }
    }
    else{
        std::cout<<"No Costmap Received\n"<<std::endl;
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}


// NEW ADDITION
std::vector<geometry_msgs::PoseStamped>
robotino_fsm_node::getDefaultPlan(const geometry_msgs::Pose end, const geometry_msgs::PosePtr start) {
    robotino_fsm::GetPlan_ getPlan;
    getPlan.request.goal.pose = end;
    getPlan.request.goal.header.frame_id = "map";
    getPlan.request.start.header.frame_id = "map";
    getPlan.request.tolerance = 0.5;
    //NEW ADDITION
    getPlan.request.costs_grid.clear();
    for(int i = 0; i< 1000*1000; i++){
        getPlan.request.costs_grid.push_back(costs_grid_default[i]);
    }
    std::cout<<"\ngetPlan called!\n";


    if (start != nullptr) {
        getPlan.request.start.pose = *start;
        //   ROS_INFO_STREAM("Path from " << getPlan.request.start.pose.position.x << " " << getPlan.request.start.pose.position.y);
    } else {
        getPlan.request.start.pose = globalRobot;
        getPlan.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(std::fmod(yaw_cam + yaw_rob, 2 * M_PI));
    }
    //   ROS_INFO_STREAM("Path to " << getPlan.request.goal.pose.position.x << " " << getPlan.request.goal.pose.position.y);

    int cnt = 0;
    while (cnt < 5) {
        if (defaultPathReq_client.call(getPlan)) {
            //      ROS_INFO_STREAM("Received path with " << getPlan.response.plan.poses.size() << " steps.");
            return getPlan.response.plan.poses;
        } else {
            cnt += 1;
        }
    }
    return std::vector<geometry_msgs::PoseStamped>{};
}

void robotino_fsm_node::init_vars() {
    tf_ok = false;
    require_new_goal = true;
    emergency = false;

    if (!nh_.getParam("is_sim", sim)) {
        ROS_WARN("no param given to is_sim");
        sim = false;
    }

    if (!nh_.getParam("kind_cost", kind_cost)) {
        ROS_WARN("no param given to kind_cost");
        kind_cost = -1;
    }
    
    if (!nh_.getParam("path_selection", world_path_param)) {
        ROS_WARN("no param given to world_path_param");
        world_path_param = -1;
    }

    // robot odom
    if (!private_nh_.getParam("robot_odom", odom_robot_topic)) {
        ROS_WARN("no robot odom topic, defaulting to /odometry/filtered");
        odom_robot_topic = "/odometry/filtered";
    }

    // camera odom
    if (!private_nh_.getParam("cam_odom", odom_cam_topic)) {
        ROS_WARN("no camera odom topic, defaulting to /camera/odometry/filtered");
        odom_cam_topic = "/camera/odometry/filtered";
    }

    // // get plan service topic
    // if (!private_nh_.getParam("get_plan", get_plan_topic)) {
    //     ROS_WARN("no topic given to call get_plan service, defaulting to /move_base/make_plan");
    //     get_plan_topic = "/move_base/make_plan";
    // }

    // // NEW ADDITION
    // if (!private_nh_.getParam("get_default_plan", get_default_plan_topic)) {
    //     ROS_WARN("no topic given to call get_plan service, defaulting to /move_base_default/make_plan");
    //     get_default_plan_topic = "/move_base_default/make_plan";
    // }

    if (!private_nh_.getParam("get_plan_costmap2D", get_plan_costmap2D_topic)) {
        ROS_WARN("no topic given to call get_plan service, defaulting to /move_base_default/make_plan");
        get_plan_costmap2D_topic = "/move_base_default/make_plan";
    }

    if (!private_nh_.getParam("get_plan_gridmap", get_plan_gridmap_topic)) {
        ROS_WARN("no topic given to call get_plan service, defaulting to /move_base_gridmap/make_plan");
        get_plan_gridmap_topic = "/move_base_gridmap/make_plan";
    }

    // get_nearby_nodes_topic
    if (!private_nh_.getParam("get_nearby_nodes", get_nearby_nodes_topic)) {
        ROS_WARN("no topic given to call get_nearby_node service, defaulting to /rtabmap/get_nearby_node_data");
        get_nearby_nodes_topic = "/rtabmap/get_nearby_node_data";
    }

    // get_frontier_topic
    if (!private_nh_.getParam("frontier_topic", get_frontier_topic)) {
        ROS_WARN("no topic given to get frontier service, defaulting to /frontier_exploration_service");
        get_frontier_topic = "/frontier_exploration_service";
    }

    // get_best_path_topic
    if (!private_nh_.getParam("best_path_topic", get_best_path_topic)) {
        ROS_WARN("no topic given to get best path, defaulting to /get_best_path");
        get_best_path_topic = "/get_best_path";
    }

    // get_best_head_topic
    if (!private_nh_.getParam("best_head_topic", get_best_head_topic)) {
        ROS_WARN("no topic given to get best path, defaulting to /get_best_head");
        get_best_head_topic = "/get_best_head";
    }

    // mpcStateTopic
    if (!private_nh_.getParam("mpcStateTopic", mpcStateTopic)) {
        ROS_WARN("no topic given to get best mpc state, defaulting to /mpc/state");
        mpcStateTopic = "/mpc/state";
    }

    // get_best_path_topic
    if (!private_nh_.getParam("mpcGoalTopic", mpcGoalTopic)) {
        ROS_WARN("no topic given to get mpcGoalTopic, defaulting to /waypoint");
        mpcGoalTopic = "/waypoint";
    }

    // max number of replan
    if (!private_nh_.getParam("max_fail_before_new_goal", max_fail_before_new_goal)) {
        ROS_WARN("no max number given to get max_fail_before_new_goal, defaulting to 5");
        max_fail_before_new_goal = 5;
    }

    if (!nh_.getParam("only_last_set", only_last_set)) {
        ROS_WARN("no param given to get only_last_set, defaulting to true");
        only_last_set = true;
    }

    if (!nh_.getParam("pre_fix", pre_fix)) {
        ROS_WARN("no param given to get pre_fix, defaulting to true");
        pre_fix = true;
    }

    if (!nh_.getParam("mid_optimizer", mid_optimizer)) {
        ROS_WARN("no param given to get mid_optimizer, defaulting to true");
        mid_optimizer = true;
    }
}

void robotino_fsm_node::transform_pose(geometry_msgs::Pose &pose) {
    tf::Transform poseToTarget;
    tf::poseMsgToTF(pose, poseToTarget);
    tf::StampedTransform map_to_odom;
    listener_.lookupTransform("/map", "odom", ros::Time::now(), map_to_odom);
    tf::poseTFToMsg(map_to_odom * poseToTarget, pose);
}

void
robotino_fsm_node::transform_odom(const nav_msgs::Odometry::ConstPtr &odomRobot,
                                  const nav_msgs::Odometry::ConstPtr &odomCam) {
    tf::Transform odom_to_target;
    tf::poseMsgToTF(odomRobot->pose.pose, odom_to_target);
    tf::StampedTransform map_to_odom;
    listener_.lookupTransform("/map", "odom", ros::Time::now(), map_to_odom);
    tf::poseTFToMsg(map_to_odom * odom_to_target, globalRobot);
    tf::poseMsgToTF(odomCam->pose.pose, odom_to_target);
    tf::poseTFToMsg(map_to_odom * odom_to_target, globalCam);
}

void robotino_fsm_node::odomCallback(const nav_msgs::Odometry::ConstPtr &odomRobot,
                                     const nav_msgs::Odometry::ConstPtr &odomCam) {

    cov_pos = odomRobot->pose.covariance.at(0) * odomRobot->pose.covariance.at(7) -
              odomRobot->pose.covariance.at(1) * odomRobot->pose.covariance.at(6);

    double roll, pitch;

    try {
        transform_odom(odomRobot, odomCam);
        tf_ok = true;
    }
    catch (tf::TransformException &e) {
        tf_ok = false;
        ROS_WARN_STREAM(e.what());
        return;
    }
    getRPY(globalRobot, roll, pitch, yaw_rob);
    getRPY(globalCam, roll, pitch, yaw_cam);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotino_fsm");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    robotino_fsm_node robotinoFsmNode(nh, private_nh);
    return 0;
}
