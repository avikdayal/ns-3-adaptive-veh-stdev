#!/bin/bash
#./waf --run "scratch/wave-simple-80211p_risk_random" > walk.csv
./waf --run "scratch/wave-80211p_risk_40n_GN_3  --p_power=$2" > walk.csv
>$1
for i in  {1..1000}
do
     grep -E "[S],[0-9]*,$i[,]|R,[0-9]*,$i\$" walk.csv >> $1
done
