#! /bin/bash
(( total = 0 ))
(( a = 0 ))
for f in *.c *.h; 
do 
echo "Processing $f file.."; 
a="$(cat $f |awk 'BEGIN{i=0}{i++}END{print i}')" ;
total=$((total+a))
done
echo "Total number of code lines: " $total
