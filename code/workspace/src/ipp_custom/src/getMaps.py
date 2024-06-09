#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8
from grid_map_msgs.msg import GridMap
import numpy as np

trial = "GCPR6"

class getMaps:
    def __init__(self):

        self.proj_map = OccupancyGrid()
        self.prob_map = OccupancyGrid()
        self.cost_map = OccupancyGrid()
        self.proj_map_data = Int8()
        self.prob_map_data = Int8()
        self.cost_map_data = Int8()

        # self.subscriber_prob = rospy.Subscriber("/rtabmap/grid_prob_map", OccupancyGrid, self.probCallback)
        # self.subscriber_proj = rospy.Subscriber("/rtabmap/proj_map", OccupancyGrid, self.projCallback)
        # self.subscriber_cost = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.costmapCallback)
        
        self.subscriber_gridMap = rospy.Subscriber("/ipp_custom/map_transformation", GridMap, self.gridMapCallback)
        self.subscriber_gridCost = rospy.Subscriber("/ipp_custom/costmap_transformation", GridMap, self.gridCostCallback)

    def gridMapCallback(self, msg):
        
        self.gridMap_map_layers = msg.layers
        self.map_gridMap_data = {}

        for i in range(len(self.gridMap_map_layers)):
            self.map_gridMap_data[self.gridMap_map_layers[i]] = msg.data[i].data
        
        np.save("gridMap_"+trial+".npy",self.map_gridMap_data)
        print("saved1")

    
    def gridCostCallback(self, msg):
        
        self.gridCost_map_layers = msg.layers
        self.gridCost_map_data = {}
        
        for i in range(len(self.gridCost_map_layers)):
            self.gridCost_map_data[self.gridCost_map_layers[i]] = msg.data[i].data
        
        np.save("gridCost_"+trial+".npy",self.gridCost_map_data)
        print("saved2")

    # def probCallback(self, msg):
        
    #     self.prob_map = msg
    #     self.prob_map_data = msg.data
    #     self.map_prob = np.array(list(self.prob_map_data))

    #     print(self.map_prob)
    #     print(self.map_prob.shape)
    #     np.save("prob_map_"+trial+".npy",self.map_prob)

    # def projCallback(self, msg):
        
    #     self.proj_map = msg
    #     self.proj_map_data = msg.data
    #     self.map_proj = np.array(list(self.proj_map_data))

    #     print(self.map_proj)
    #     print(self.map_proj.shape)
    #     np.save("proj_map_"+trial+".npy",self.map_proj)

    # def costmapCallback(self, msg):

    #     self.cost_map = msg
    #     self.cost_map_data = msg.data
    #     self.map_cost = np.array(list(self.cost_map_data))

    #     print(self.map_cost)
    #     print(self.map_cost.shape)
    #     np.save("cost_map_"+trial+".npy",self.map_cost)

if __name__ == "__main__":
    rospy.init_node('getMaps_node')
    ma = getMaps()
    while not rospy.is_shutdown():
        rospy.spin()
        break
        # hello_str = "hello world"
        # rospy.loginfo(hello_str)
