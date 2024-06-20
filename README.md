# METAS

ROS workspace with the code to run everything including evaluatory codes is added in in code/workspace, with details in the METAS.pdf.

Note: the code might not be working correctly due to the anonimization procedure.

Results are available here: https://drive.google.com/file/d/1IRtgg_3bcaTNUl91YRyxjJY8iOgpzz3e/view?usp=sharing

## Instructions to install the repository:

We use ROS Noetic to build our package. Since METAS is developed using the iRotate package, we follow the same [installation instructions](https://github.com/eliabntt/irotate_active_slam/blob/noetic/INSTALL.md).

Other than the prerequites of iRotate, the following are required for METAS package to:

- numpy
- scipy
- grid_map ROS package (already in workspace, no need to install seperately)

after following the installation steps of iRotate, and installing all prerequites, we can build the workspace using:

```catkin build -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release -j8```