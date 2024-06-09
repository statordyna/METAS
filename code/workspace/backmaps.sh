#/bin/bash
IFS='/'
str="$*"
ls /media/statordyna/Backup\ Plus/Test/"$str"
sleep 2
mv -i *.npy /media/statordyna/Backup\ Plus/Test/"$str"

