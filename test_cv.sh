#!/bin/bash
./waf --run "scratch/wave-bsm-risk_cv --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_cv.output.csv --CSVfileName2=bsm_20node_cv.output2.csv --CSVfileName3=bsm_20node_cv.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=20 --CSVfileName=bsm_20node_cv.output.csv --CSVfileName2=bsm_20node_cv.output.csv"
./waf --run "scratch/wave-bsm-risk_cv --protocol=0 --totaltime=60 --nodes=40 --CSVfileName=bsm_40node_cv.output.csv --CSVfileName2=bsm_40node_cv.output2.csv --CSVfileName3=bsm_40node_cv.output3.csv"
./waf --run "scratch/wave-bsm-risk_cv --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_cv.output.csv --CSVfileName2=bsm_80node_cv.output2.csv --CSVfileName3=bsm_80node_cv.output3.csv"
./waf --run "scratch/wave-bsm-risk_cv --protocol=0 --totaltime=60 --nodes=120 --CSVfileName=bsm_120node_cv.output.csv --CSVfileName2=bsm_120node_cv.output2.csv --CSVfileName3=bsm_120node_cv.output3.csv"
#./waf --run "scratch/wave-bsm-risk --protocol=0 --totaltime=60 --nodes=80 --CSVfileName=bsm_80node_cv.output.csv --CSVfileName2=bsm_80node_cv.output.csv"
./waf --run "scratch/wave-bsm-risk_cv --protocol=0 --totaltime=60 --nodes=160 --CSVfileName=bsm_160node_cv.output.csv --CSVfileName2=bsm_160node_cv.output2.csv --CSVfileName3=bsm_160node_cv.output3.csv"
