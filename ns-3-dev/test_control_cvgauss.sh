#!/bin/bash
./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=20 --pcap=0 --CSVfileName=bsm_control_20node_cvgauss.output.csv --CSVfileName2=bsm_control_20node_cvgauss.output2.csv --CSVfileName3=bsm_control_20node_cvgauss.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_cvgauss.output.csv --CSVfileName2=bsm_20node_cvgauss.output.csv"
./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=40 --pcap=0 --CSVfileName=bsm_control_40node_cvgauss.output.csv --CSVfileName2=bsm_control_40node_cvgauss.output2.csv --CSVfileName3=bsm_control_40node_cvgauss.output3.csv"
./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=80 --pcap=0 --CSVfileName=bsm_control_80node_cvgauss.output.csv --CSVfileName2=bsm_control_80node_cvgauss.output2.csv --CSVfileName3=bsm_control_80node_cvgauss.output3.csv"
./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=120 --pcap=0 --CSVfileName=bsm_control_120node_cvgauss.output.csv --CSVfileName2=bsm_control_120node_cvgauss.output2.csv --CSVfileName3=bsm_control_120node_cvgauss.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node.output.csv --CSVfileName2=bsm_80node.output.csv"
./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=160 --pcap=0 --CSVfileName=bsm_control_160node_cvgauss.output.csv --CSVfileName2=bsm_control_160node_cvgauss.output2.csv --CSVfileName3=bsm_control_160node_cvgauss.output3.csv"
