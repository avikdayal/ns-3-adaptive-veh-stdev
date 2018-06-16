#!/bin/bash
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_5sec.output.csv --CSVfileName2=bsm_20node_5sec.output.csv"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=40"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_5sec.output.csv --CSVfileName2=bsm_80node_5sec.output.csv"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=160"
