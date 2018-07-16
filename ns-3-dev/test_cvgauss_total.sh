#!/bin/bash
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=20 --CSVfileName=bsm_20node_cvgauss.output.csv --CSVfileName2=bsm_20node_cvgauss.output2.csv --CSVfileName3=bsm_20node_cvgauss.output3.csv" > trace_20n.log 2> bsm_20n.log
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=20 --pcap=0 --CSVfileName=bsm_control_20node_cvgauss.output.csv --CSVfileName2=bsm_control_20node_cvgauss.output2.csv --CSVfileName3=bsm_control_20node_cvgauss.output3.csv" > trace_c20n.log 2> bsm_c20n.log
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_cvgauss.output.csv --CSVfileName2=bsm_20node_cvgauss.output.csv"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=40 --CSVfileName=bsm_40node_cvgauss.output.csv --CSVfileName2=bsm_40node_cvgauss.output2.csv --CSVfileName3=bsm_40node_cvgauss.output3.csv" > trace_40n.log 2> bsm_40n.log
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=40 --pcap=0 --CSVfileName=bsm_control_40node_cvgauss.output.csv --CSVfileName2=bsm_control_40node_cvgauss.output2.csv --CSVfileName3=bsm_control_40node_cvgauss.output3.csv" > trace_c40n.log 2> bsm_c40n.log

echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=80 --CSVfileName=bsm_80node_cvgauss.output.csv --CSVfileName2=bsm_80node_cvgauss.output2.csv --CSVfileName3=bsm_80node_cvgauss.output3.csv" > trace_80n.log 2> bsm_80n.log
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=80 --pcap=0 --CSVfileName=bsm_control_80node_cvgauss.output.csv --CSVfileName2=bsm_control_80node_cvgauss.output2.csv --CSVfileName3=bsm_control_80node_cvgauss.output3.csv"> trace_c80n.log 2> bsm_c80n.log

echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=120 --CSVfileName=bsm_120node_cvgauss.output.csv --CSVfileName2=bsm_120node_cvgauss.output2.csv --CSVfileName3=bsm_120node_cvgauss.output3.csv" > trace_120n.log 2> bsm_120n.log
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=120 --pcap=0 --CSVfileName=bsm_control_120node_cvgauss.output.csv --CSVfileName2=bsm_control_120node_cvgauss.output2.csv --CSVfileName3=bsm_control_120node_cvgauss.output3.csv" > trace_c120n.log 2> bsm_c120n.log
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_cvgauss.output.csv --CSVfileName2=bsm_80node_cvgauss.output.csv"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=160 --CSVfileName=bsm_160node_cvgauss.output.csv --CSVfileName2=bsm_160node_cvgauss.output2.csv --CSVfileName3=bsm_160node_cvgauss.output3.csv" > trace_160n.log 2> bsm_160n.log
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=1000 --nodes=160 --pcap=0 --CSVfileName=bsm_control_160node_cvgauss.output.csv --CSVfileName2=bsm_control_160node_cvgauss.output2.csv --CSVfileName3=bsm_control_160node_cvgauss.output3.csv"> trace_c160n.log 2> bsm_c160n.log
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=200 --CSVfileName=bsm_200node_cvgauss.output.csv --CSVfileName2=bsm_200node_cvgauss.output2.csv --CSVfileName3=bsm_200node_cvgauss.output3.csv"
