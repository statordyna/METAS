#/bin/bash
sleep 1
source ./devel/setup.bash
rosrun robotino_simulations convert_map2
rosrun robotino_simulations calculate_map_error2
IFS='/'
str="$*"
rosrun map_server map_saver -f map
mv map.yaml /media/statordyna/Backup\ Plus/Test/"$str"/
mv map.pgm /media/statordyna/Backup\ Plus/Test/"$str"/
mv /media/statordyna/Backup\ Plus/Test/E2/occupancy.txt /media/statordyna/Backup\ Plus/Test/"$str"
mv /media/statordyna/Backup\ Plus/Test/E2/general_results.txt /media/statordyna/Backup\ Plus/Test/"$str"

cp ~/.ros/rtabmap.db rtabmap.db && rtabmap-report --poses ./rtabmap.db
