#/bin/bash
IFS='/'
str="$*"
ls /media/statordyna/Backup\ Plus/Test/"$str"
./map.sh "$str"
python3 evaluate_ate.py rtabmap_slam.txt rtabmap_gt.txt --plot ate.pdf --verbose > ate.txt
python3 evaluate_rpe.py rtabmap_odom.txt rtabmap_gt.txt --plot rpe.pdf --fixed_delta  --verbose > rpe.txt
mv -i *.txt /media/statordyna/Backup\ Plus/Test/"$str"
mv -i *.npy /media/statordyna/Backup\ Plus/Test/"$str"
mv -i rtabmap.db /media/statordyna/Backup\ Plus/Test/"$str"
mv -i ate.pdf /media/statordyna/Backup\ Plus/Test/"$str"
mv -i rpe.pdf /media/statordyna/Backup\ Plus/Test/"$str"
