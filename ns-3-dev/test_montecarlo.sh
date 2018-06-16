#!/bin/bash

for i in  {4..9}
do
  NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_5sec$i.output.csv --CSVfileName2=bsm_20node_5sec$i.output2.csv --CSVfileName3=bsm_20node_5sec$i.output3.csv"
     #grep -E "[S],[0-9]*,$i[,]|R,[0-9]*,$i\$" walk.csv >> $1
done
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_5sec.output.csv --CSVfileName2=bsm_20node_5sec.output.csv"
for i in  {4..9}
do
NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=40 --CSVfileName=bsm_40node_5sec$i.output.csv --CSVfileName2=bsm_40node_5sec$i.output2.csv --CSVfileName3=bsm_40node_5sec$i.output3.csv"
done
for i in  {4..9}
do
NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_5sec$i.output.csv --CSVfileName2=bsm_80node_5sec$i.output2.csv --CSVfileName3=bsm_80node_5sec$i.output3.csv"
done
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_5sec.output.csv --CSVfileName2=bsm_80node_5sec.output.csv"
for i in  {4..9}
do
NS_GLOBAL_VALUE="RngRun=$i" ./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=160 --CSVfileName=bsm_160node_5sec$i.output.csv --CSVfileName2=bsm_160node_5sec$i.output2.csv --CSVfileName3=bsm_160node_5sec$i.output3.csv"
done
