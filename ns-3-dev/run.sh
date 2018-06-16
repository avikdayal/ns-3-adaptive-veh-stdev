#!/bin/bash
#./waf --run "scratch/wave-simple-80211p_risk_random" > walk.csv
#./waf --run "scratch/wave-80211p_risk_40n_GN_4  --p_power=$2" > walk.csv
rm -f $1.unsorted.csv $1.csv
./waf --run "scratch/wave-80211p_risk_40n_GN_5  --p_power=$2" > $1.unsorted.csv
#./waf --run "scratch/wave-80211p_risk_40n_GN_4  --p_power=$2 --num_nodes=$3" > walk.csv
>$1.csv
for i in  {1..10000}
do
     grep -E "[S],[0-9]*,$i[,]|R,[0-9]*,$i\$|R,[0-9]*,$i," $1.unsorted.csv >> $1.csv
done
