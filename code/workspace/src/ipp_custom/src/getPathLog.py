#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8
import numpy as np
from active_slam.msg import GroupPathInfo

class getPah:
    def __init__(self):

        self.subs_costmap2D = rospy.Subscriber("/grouped_costmap2D_paths_info", GroupPathInfo, self.costmap2DCallback)
        self.subs_gridmap_default = rospy.Subscriber("/grouped_gridmap_default_paths_info", GroupPathInfo, self.gridmap_defaultCallback)
        self.subs_gridmap_transformation = rospy.Subscriber("/grouped_gridmap_transformation_paths_info", GroupPathInfo, self.gridmap_transformationCallback)

        self.gridmap_transformation_data = []
        self.gridmap_default_data = []
        self.gridmap_costmap2D_data = []
    
    def costmap2DCallback(self, msg):
        
        data = []

        time = msg.stamp.secs + (10**-9)*msg.stamp.nsecs
        no = msg.no_of_paths

        data.append(time)
        data.append(no)

        if no > 0:
            gp = []
            for i in range(no):
                d_ = []
                d_.append(msg.grouped_paths[i].entropy)
                d_.append(msg.grouped_paths[i].entropy_free)
                d_.append(msg.grouped_paths[i].length)
                d_.append(msg.grouped_paths[i].x)
                d_.append(msg.grouped_paths[i].y)
                d_.append(msg.grouped_paths[i].no_of_waypoints)

                gp.append(d_)

            data.append(gp)

            bp = []
            bp.append(msg.best_path.entropy)
            bp.append(msg.best_path.entropy_free)
            bp.append(msg.best_path.length)
            bp.append(msg.best_path.x)
            bp.append(msg.best_path.y)
            bp.append(msg.best_path.no_of_waypoints)
            
            data.append(bp)

        else:
            data.append([])
            data.append([])

        self.gridmap_costmap2D_data.append(data)

        print("costmap2D_path")
        np.save("costmap2D_path.npy",np.array(ma.gridmap_costmap2D_data, dtype="object"))

    def gridmap_defaultCallback(self, msg):
        
        data = []

        time = msg.stamp.secs + (10**-9)*msg.stamp.nsecs
        no = msg.no_of_paths

        data.append(time)
        data.append(no)

        if no > 0:
            gp = []
            for i in range(no):
                d_ = []
                d_.append(msg.grouped_paths[i].entropy)
                d_.append(msg.grouped_paths[i].entropy_free)
                d_.append(msg.grouped_paths[i].length)
                d_.append(msg.grouped_paths[i].x)
                d_.append(msg.grouped_paths[i].y)
                d_.append(msg.grouped_paths[i].no_of_waypoints)

                gp.append(d_)

            data.append(gp)

            bp = []
            bp.append(msg.best_path.entropy)
            bp.append(msg.best_path.entropy_free)
            bp.append(msg.best_path.length)
            bp.append(msg.best_path.x)
            bp.append(msg.best_path.y)
            bp.append(msg.best_path.no_of_waypoints)
            
            data.append(bp)

        else:
            data.append([])
            data.append([])

        self.gridmap_default_data.append(data)

        print("gridmap_default_path")
        np.save("gridmap_default_path.npy",np.array(ma.gridmap_default_data, dtype="object"))

    def gridmap_transformationCallback(self, msg):
        data = []

        time = msg.stamp.secs + (10**-9)*msg.stamp.nsecs
        no = msg.no_of_paths

        data.append(time)
        data.append(no)

        if no > 0:
            gp = []
            for i in range(no):
                d_ = []
                d_.append(msg.grouped_paths[i].entropy)
                d_.append(msg.grouped_paths[i].entropy_free)
                d_.append(msg.grouped_paths[i].length)
                d_.append(msg.grouped_paths[i].x)
                d_.append(msg.grouped_paths[i].y)
                d_.append(msg.grouped_paths[i].no_of_waypoints)

                gp.append(d_)

            data.append(gp)

            bp = []
            bp.append(msg.best_path.entropy)
            bp.append(msg.best_path.entropy_free)
            bp.append(msg.best_path.length)
            bp.append(msg.best_path.x)
            bp.append(msg.best_path.y)
            bp.append(msg.best_path.no_of_waypoints)
            
            data.append(bp)

        else:
            data.append([])
            data.append([])

        self.gridmap_transformation_data.append(data)
        np.save("gridmap_transformation_path.npy",np.array(ma.gridmap_transformation_data, dtype="object"))
        print("gridmap_transformation_path")


if __name__ == "__main__":
    rospy.init_node('getMaps_node')
    ma = getPah()

    while not rospy.is_shutdown():
        rospy.spin()

    print(ma.gridmap_transformation_data)
    print(ma.gridmap_default_data)

    # np.save("gridmap_transformation_path.npy",np.array(ma.gridmap_transformation_data, dtype="object"))
    # np.save("gridmap_default_path2.npy",np.array(ma.gridmap_default_data, dtype="object"))
    
    print("Logging..")