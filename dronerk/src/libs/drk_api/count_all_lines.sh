#! /bin/bash
(( total = 0 ))
(( a = 0 ))
for f in *.c $DRK_ROOT/include/drk/*.h; 
do 
a="$(cat $f |awk 'BEGIN{i=0}{i++}END{print i}')" ;
echo "Processing $f file.." $a "lines" 
total=$((total+a))
done
echo "Total number of code lines: " $total
