#/bin/bash
sleep 1
source ./devel/setup.bash
rosrun robotino_simulations convert_map
rosrun robotino_simulations calculate_map_error
IFS='/'
str="$*"
rosrun map_server map_saver -f map
mv map.yaml /media/statordyna/Backup\ Plus/Test/"$str"/
mv map.pgm /media/statordyna/Backup\ Plus/Test/"$str"/
mv /media/statordyna/Backup\ Plus/Test/E1/occupancy.txt /media/statordyna/Backup\ Plus/Test/"$str"
mv /media/statordyna/Backup\ Plus/Test/E1/general_results.txt /media/statordyna/Backup\ Plus/Test/"$str"

cp ~/.ros/rtabmap.db rtabmap.db && rtabmap-report --poses ./rtabmap.db
