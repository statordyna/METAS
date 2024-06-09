#!/usr/bin/env python3

import rospy
from time import time
from ipp_custom.srv import transferMap, transferMapRequest, transferMapResponse

import numpy as np
from scipy import ndimage

entropy_thresholding = []
filtering = []
distancing = []
log_transformation = []

def getCostmap(req):
    # gets [1, 621x621] entropy map from transformation.cpp
    # return the [1, 621x621] costmap

    resp = transferMapResponse()

    data = np.array(req.entropy_map)/100.0
    data = data.reshape((621,621))
    
    ent = np.copy(data)

    start = time()
    ent[data>0.7] = 1
    ent[data<=0.7] = 0
    ent[data==-1] = -1
    end = time()
    entropy_thresholding.append((end - start))

    start = time()
    res = ndimage.median_filter(ent, (3,3))
    end = time()
    filtering.append((end - start))
    
    res2 = np.copy(res)
    
    start = time()
    res[res2 == 0] = 1
    res[res2 == 1] = 0
    res[res2 == -1] = 0
    res_dt = 0.05*ndimage.distance_transform_edt(res)
    end = time()
    distancing.append((end - start))

    start = time()
    for i in range(621):
        for j in range(621):
            if data[i][j] == -1:
                res_dt[i][j] = -1
            if res_dt[i][j] >= 0:
                res_dt[i][j] = abs(res_dt[i][j] - 1.5)
                res_dt[i][j] = int(500*np.log10(res_dt[i][j]/0.1 +1))
    end = time()
    log_transformation.append((end - start))

    print("\nTrial: = "+ str(len(entropy_thresholding)))
    print("Entropy Thresholding:\t", np.mean(np.array(entropy_thresholding)), np.std(np.array(entropy_thresholding)))
    print("Median Filtering:\t", np.mean(np.array(filtering)), np.std(np.array(filtering)))
    print("Distance Transform:\t", np.mean(np.array(distancing)), np.std(np.array(distancing)))
    print("Log Transformation:\t", np.mean(np.array(log_transformation)), np.std(np.array(log_transformation)))

    resp.costmap = []
    for x in res_dt.flatten().tolist():
        resp.costmap.append(int(x))
    
    resp.costmap = tuple(resp.costmap)

    return resp

if __name__ == "__main__":
    rospy.init_node('transformation_transfer')
    s = rospy.Service('getCostmapTransfer', transferMap, getCostmap)
    print("RosService Ready.")
    rospy.spin()