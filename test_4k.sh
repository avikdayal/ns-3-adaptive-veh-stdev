#!/bin/bash
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_5sec_4k.output.csv --CSVfileName2=bsm_20node_5sec_4k.output2.csv --CSVfileName3=bsm_20node_5sec_4k.output3.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=40 --CSVfileName=bsm_40node_5sec_4k.output.csv --CSVfileName2=bsm_40node_5sec_4k.output2.csv --CSVfileName3=bsm_40node_5sec_4k.output3.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_5sec_4k.output.csv --CSVfileName2=bsm_80node_5sec_4k.output2.csv --CSVfileName3=bsm_80node_5sec_4k.output3.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=160 --CSVfileName=bsm_160node_5sec_4k.output.csv --CSVfileName2=bsm_160node_5sec_4k.output2.csv --CSVfileName3=bsm_160node_5sec_4k.output3.csv"



./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=20 --pcap=0 --CSVfileName=bsm_control_20node_4k.output.csv --CSVfileName2=bsm_control_20node_4k.output2.csv --CSVfileName3=bsm__control_20node_4k.output3.csv"
#./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node.output.csv --CSVfileName2=bsm_20node.output.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=40 --pcap=0 --CSVfileName=bsm_control_40node_4k.output.csv --CSVfileName2=bsm_control_40node_4k.output2.csv --CSVfileName3=bsm__control_40node_4k.output3.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=80 --pcap=0 --CSVfileName=bsm_control_80node_4k.output.csv --CSVfileName2=bsm_control_80node_4k.output2.csv --CSVfileName3=bsm__control_80node_4k.output3.csv"
#./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node.output.csv --CSVfileName2=bsm_80node.output.csv"
./waf --run "scratch/wave-bsm-risk_4k --protocol=0 --totaltime=60 --nodes=160 --pcap=0 --CSVfileName=bsm_control_160node_4k.output.csv --CSVfileName2=bsm_control_160node_4k.output2.csv --CSVfileName3=bsm__control_160node_4k.output3.csv"
