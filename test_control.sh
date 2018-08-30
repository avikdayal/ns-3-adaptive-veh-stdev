#!/bin/bash
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --pcap=0 --CSVfileName=bsm_control_20node.output.csv --CSVfileName2=bsm_control_20node.output2.csv --CSVfileName3=bsm__control_20node.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node.output.csv --CSVfileName2=bsm_20node.output.csv"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=40 --pcap=0 --CSVfileName=bsm_control_40node.output.csv --CSVfileName2=bsm_control_40node.output2.csv --CSVfileName3=bsm__control_40node.output3.csv"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --pcap=0 --CSVfileName=bsm_control_80node.output.csv --CSVfileName2=bsm_control_80node.output2.csv --CSVfileName3=bsm__control_80node.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node.output.csv --CSVfileName2=bsm_80node.output.csv"
./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=160 --pcap=0 --CSVfileName=bsm_control_160node.output.csv --CSVfileName2=bsm_control_160node.output2.csv --CSVfileName3=bsm__control_160node.output3.csv"
