#!/bin/bash

#echo "about to start 20 node adaptive"
#echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 16.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=20 --CSVfileName=bsm_20node_cvgauss.output.csv --CSVfileName2=bsm_20node_cvgauss.output2.csv --CSVfileName3=bsm_20node_cvgauss.output3.csv" > trace_20n.log 2> bsm_20n.log

#echo "about to start 20 node control"
#echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 16.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=20 --pcap=0 --CSVfileName=bsm_control_20node_cvgauss.output.csv --CSVfileName2=bsm_control_20node_cvgauss.output2.csv --CSVfileName3=bsm_control_20node_cvgauss.output3.csv" > trace_c20n.log 2> bsm_c20n.log


#echo "about to start 40 node adaptive"
#echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 16.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=40 --CSVfileName=bsm_40node_cvgauss.output.csv --CSVfileName2=bsm_40node_cvgauss.output2.csv --CSVfileName3=bsm_40node_cvgauss.output3.csv" > trace_40n.log 2> bsm_40n.log
#echo "about to start 40 node control"
#echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 16.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#S_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=40 --pcap=0 --CSVfileName=bsm_control_40node_cvgauss.output.csv --CSVfileName2=bsm_control_40node_cvgauss.output2.csv --CSVfileName3=bsm_control_40node_cvgauss.output3.csv" > trace_c40n.log 2> bsm_c40n.log


#echo "about to start 60 node adaptive"
#echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=60 --CSVfileName=bsm_60node_cvgauss.output.csv --CSVfileName2=bsm_60node_cvgauss.output2.csv --CSVfileName3=bsm_60node_cvgauss.output3.csv" > trace_60n.log 2> bsm_60n.log


#echo "about to start 60 node control"
#echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=60 --pcap=0 --CSVfileName=bsm_control_60node_cvgauss.output.csv --CSVfileName2=bsm_control_60node_cvgauss.output2.csv --CSVfileName3=bsm_control_60node_cvgauss.output3.csv"> trace_c60n.log 2> bsm_c60n.log


echo "about to start 70 node adaptive"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 3" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=70 --CSVfileName=bsm_70node_cvgauss.output.csv --CSVfileName2=bsm_70node_cvgauss.output2.csv --CSVfileName3=bsm_70node_cvgauss.output3.csv" > trace_70n.log 2> bsm_70n.log


echo "about to start 70 node control"
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 3" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=70 --pcap=0 --CSVfileName=bsm_control_70node_cvgauss.output.csv --CSVfileName2=bsm_control_70node_cvgauss.output2.csv --CSVfileName3=bsm_control_70node_cvgauss.output3.csv"> trace_c70n.log 2> bsm_c70n.log

echo "about to start 80 node adaptive"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=80 --CSVfileName=bsm_80node_cvgauss.output.csv --CSVfileName2=bsm_80node_cvgauss.output2.csv --CSVfileName3=bsm_80node_cvgauss.output3.csv" > trace_80n.log 2> bsm_80n.log


echo "about to start 80 node control"
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=80 --pcap=0 --CSVfileName=bsm_control_80node_cvgauss.output.csv --CSVfileName2=bsm_control_80node_cvgauss.output2.csv --CSVfileName3=bsm_control_80node_cvgauss.output3.csv"> trace_c80n.log 2> bsm_c80n.log

echo "about to start 90 node adaptive"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=90 --CSVfileName=bsm_90node_cvgauss.output.csv --CSVfileName2=bsm_90node_cvgauss.output2.csv --CSVfileName3=bsm_90node_cvgauss.output3.csv" > trace_90n.log 2> bsm_90n.log


echo "about to start 90 node control"
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=90 --pcap=0 --CSVfileName=bsm_control_80node_cvgauss.output.csv --CSVfileName2=bsm_control_90node_cvgauss.output2.csv --CSVfileName3=bsm_control_90node_cvgauss.output3.csv"> trace_c90n.log 2> bsm_c90n.log


echo "about to start 100 node adaptive"
echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=100 --CSVfileName=bsm_100node_cvgauss.output.csv --CSVfileName2=bsm_100node_cvgauss.output2.csv --CSVfileName3=bsm_100node_cvgauss.output3.csv" > trace_100n.log 2> bsm_100n.log

echo "about to start 100 node control"
echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
echo "#define STD_DEV 2" >> "src/wave/model/adaptive_priotag.h"
NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=100 --nodes=100 --pcap=0 --CSVfileName=bsm_control_100node_cvgauss.output.csv --CSVfileName2=bsm_control_100node_cvgauss.output2.csv --CSVfileName3=bsm_control_100node_cvgauss.output3.csv"> trace_c100n.log 2> bsm_c100n.log

#echo "about to start 120 node adaptive"
#echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=120 --CSVfileName=bsm_120node_cvgauss.output.csv --CSVfileName2=bsm_120node_cvgauss.output2.csv --CSVfileName3=bsm_120node_cvgauss.output3.csv" > trace_120n.log 2> bsm_120n.log


#echo "about to start 120 node control"
#echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=120 --pcap=0 --CSVfileName=bsm_control_120node_cvgauss.output.csv --CSVfileName2=bsm_control_120node_cvgauss.output2.csv --CSVfileName3=bsm_control_120node_cvgauss.output3.csv" > trace_c120n.log 2> bsm_c120n.log

#echo "about to start 160 node adaptive"
#echo "#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=160 --CSVfileName=bsm_160node_cvgauss.output.csv --CSVfileName2=bsm_160node_cvgauss.output2.csv --CSVfileName3=bsm_160node_cvgauss.output3.csv" > trace_160n.log 2> bsm_160n.log

#echo "about to start 160 node control"
#echo "//#define ADAPTIVE_PRIO" > "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define TTC_THRESH_UPPER 15.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define CRASH_THRESHOLD 4.8" >> "src/wave/model/adaptive_priotag.h"
#echo "#define PRIO_MANUEVER_THRESHOLD 5.0" >> "src/wave/model/adaptive_priotag.h"
#echo "#define MEAN_NODE_SPEED 25" >> "src/wave/model/adaptive_priotag.h"
#echo "#define STD_DEV 1" >> "src/wave/model/adaptive_priotag.h"
#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=50 --nodes=160 --pcap=0 --CSVfileName=bsm_control_160node_cvgauss.output.csv --CSVfileName2=bsm_control_160node_cvgauss.output2.csv --CSVfileName3=bsm_control_160node_cvgauss.output3.csv"> trace_c160n.log 2> bsm_c160n.log




#NS_GLOBAL_VALUE="RngRun=$5" ./waf --run "scratch/wave-bsm-risk_cvgauss --protocol=0 --totaltime=60 --nodes=200 --CSVfileName=bsm_200node_cvgauss.output.csv --CSVfileName2=bsm_200node_cvgauss.output2.csv --CSVfileName3=bsm_200node_cvgauss.output3.csv"
