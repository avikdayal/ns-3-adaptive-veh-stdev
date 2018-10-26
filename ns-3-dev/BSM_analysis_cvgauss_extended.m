%% Import data from text file.
% Script for importing data from the following text file:
%
%    /Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-RA-simulation/ns-3-dev/bsm_10node.output2.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2018/05/01 12:59:04
clear all;
close all;

load hp_PDR_SR_1_control;
load hp_PDR_SR_3_control;
load hp_PDR_SR_5_control;
load hp_PDR_SR_7_control;
%% Initialize variables.
filename_20n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_20node_cvgauss.output2.csv';
filename_40n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_40node_cvgauss.output2.csv';
filename_60n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_60node_cvgauss.output2.csv';
filename_70n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_70node_cvgauss.output2.csv';
filename_80n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_80node_cvgauss.output2.csv';
filename_90n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_90node_cvgauss.output2.csv';
filename_100n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_100node_cvgauss.output2.csv';
filename_120n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_120node_cvgauss.output2.csv';
filename_160n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_160node_cvgauss.output2.csv';

filename_20n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_20node_cvgauss.output3.csv';
filename_40n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_40node_cvgauss.output3.csv';
filename_60n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_60node_cvgauss.output3.csv';
filename_70n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_70node_cvgauss.output3.csv';
filename_80n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_80node_cvgauss.output3.csv';
filename_90n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_90node_cvgauss.output3.csv';
filename_100n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_100node_cvgauss.output3.csv';
filename_120n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_120node_cvgauss.output3.csv';
filename_160n_2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_160node_cvgauss.output2.csv';

filename_20n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_20node_cvgauss.output3.csv';
filename_40n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_40node_cvgauss.output3.csv';
filename_60n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_60node_cvgauss.output3.csv';
filename_70n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_70node_cvgauss.output3.csv';
filename_80n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_80node_cvgauss.output3.csv';
filename_90n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_90node_cvgauss.output3.csv';
filename_100n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_100node_cvgauss.output3.csv';
filename_120n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_120node_cvgauss.output3.csv';
filename_160n_c2 = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_control_160node_cvgauss.output3.csv';

filename_20n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_20node_cvgauss.output2.csv';
filename_40n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_40node_cvgauss.output2.csv';
filename_60n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_60node_cvgauss.output2.csv';
filename_70n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_70node_cvgauss.output2.csv';
filename_80n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_80node_cvgauss.output2.csv';
filename_90n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_90node_cvgauss.output2.csv';
filename_100n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_100node_cvgauss.output2.csv';
filename_120n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_120node_cvgauss.output2.csv';
filename_160n_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_160node_cvgauss.output2.csv';

filename_20n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh/ns-3-dev/bsm_20node_cvgauss.output3.csv';
filename_40n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_40node_cvgauss.output3.csv';
filename_60n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_60node_cvgauss.output3.csv';
filename_70n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_70node_cvgauss.output3.csv';
filename_80n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_80node_cvgauss.output3.csv';
filename_90n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_90node_cvgauss.output3.csv';
filename_100n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_100node_cvgauss.output3.csv';
filename_120n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_120node_cvgauss.output3.csv';
filename_160n_2_50ms = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-adaptive-veh-50ms/ns-3-dev/bsm_160node_cvgauss.output3.csv';



%filename_200n = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-beacon-rate/ns-3-dev/bsm_160node_cvgauss.output2.csv';

delimiter = ',';
startRow = 2;

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
%	column12: double (%f)
%   column13: double (%f)
%	column14: double (%f)
%   column15: double (%f)
%	column16: double (%f)
%   column17: double (%f)
%	column18: double (%f)
%   column19: double (%f)
%	column20: double (%f)
%   column21: double (%f)
%	column22: double (%f)
%   column23: double (%f)
%	column24: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
formatSpec_40n = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
%formatSpec_80n = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
formatSpec_80n = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
formatSpec_120n = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
formatSpec_160n = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f';
formatSpec_2 = '%f%f%f%f%f%f%f%f%f%[^\n\r]';
%% Open the text file.
fileID = fopen(filename_20n,'r');
fileID_40n = fopen(filename_40n,'r');
fileID_60n = fopen(filename_60n,'r');
fileID_70n = fopen(filename_70n,'r');
fileID_80n = fopen(filename_80n,'r');
fileID_90n = fopen(filename_90n,'r');
fileID_100n = fopen(filename_100n,'r');
fileID_120n = fopen(filename_120n,'r');
fileID_160n = fopen(filename_160n,'r');
%% Open the text file.
fileID_2 = fopen(filename_20n_2,'r');
fileID_40n_2 = fopen(filename_40n_2,'r');
fileID_60n_2 = fopen(filename_60n_2,'r');
fileID_70n_2 = fopen(filename_70n_2,'r');
fileID_80n_2 = fopen(filename_80n_2,'r');
fileID_90n_2 = fopen(filename_90n_2,'r');
fileID_100n_2 = fopen(filename_100n_2,'r');
fileID_120n_2 = fopen(filename_120n_2,'r');
fileID_160n_2 = fopen(filename_160n_2,'r');

fileID_c2 = fopen(filename_20n_c2,'r');
fileID_40n_c2 = fopen(filename_40n_c2,'r');
fileID_60n_c2 = fopen(filename_60n_c2,'r');
fileID_70n_c2 = fopen(filename_70n_c2,'r');
fileID_80n_c2 = fopen(filename_80n_c2,'r');
fileID_90n_c2 = fopen(filename_90n_c2,'r');
fileID_100n_c2 = fopen(filename_100n_c2,'r');
fileID_120n_c2 = fopen(filename_120n_c2,'r');
fileID_160n_c2 = fopen(filename_160n_c2,'r');
%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_40n = textscan(fileID_40n, formatSpec_40n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_60n = textscan(fileID_60n, formatSpec_40n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_70n = textscan(fileID_70n, formatSpec_40n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_80n = textscan(fileID_80n, formatSpec_80n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_90n = textscan(fileID_90n, formatSpec_80n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_100n = textscan(fileID_100n, formatSpec_80n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_120n = textscan(fileID_120n, formatSpec_160n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_160n = textscan(fileID_160n, formatSpec_160n, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

dataArray_2 = textscan(fileID_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_40n_2 = textscan(fileID_40n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_60n_2 = textscan(fileID_60n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_70n_2 = textscan(fileID_70n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_80n_2 = textscan(fileID_80n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_90n_2 = textscan(fileID_90n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_100n_2 = textscan(fileID_100n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_120n_2 = textscan(fileID_120n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_160n_2 = textscan(fileID_160n_2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_c2 = textscan(fileID_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_40n_c2 = textscan(fileID_40n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_60n_c2 = textscan(fileID_60n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_70n_c2 = textscan(fileID_70n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

dataArray_80n_c2 = textscan(fileID_80n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_90n_c2 = textscan(fileID_90n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_100n_c2 = textscan(fileID_100n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

dataArray_120n_c2 = textscan(fileID_120n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
dataArray_160n_c2 = textscan(fileID_160n_c2, formatSpec_2, 'Delimiter', delimiter, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');


%% Close the text file.
fclose(fileID);
fclose(fileID_40n);
fclose(fileID_80n);
fclose(fileID_120n);
fclose(fileID_160n);
fclose(fileID_2);
fclose(fileID_40n_2);
fclose(fileID_80n_2);
fclose(fileID_120n_2);
fclose(fileID_160n_2);
fclose(fileID_c2);
fclose(fileID_40n_c2);
fclose(fileID_80n_c2);
fclose(fileID_120n_c2);
fclose(fileID_160n_c2);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
%safety_range=[50 100 150 200 250 300 350 400 450 500];
safety_range=[25 50 100 200 250 300 350 400 450 500];

BSM_PDR1 = dataArray{:, 1};
BSM_PDR2 = dataArray{:, 2};
BSM_PDR3 = dataArray{:, 3};
BSM_PDR4 = dataArray{:, 4};
BSM_PDR5 = dataArray{:, 5};
BSM_PDR6 = dataArray{:, 6};
BSM_PDR7 = dataArray{:, 7};
BSM_PDR8 = dataArray{:, 8};
BSM_PDR9 = dataArray{:, 9};
BSM_PDR10 = dataArray{:, 10};
HPBSM_PDR1 = dataArray{:, 11};
HPBSM_PDR2 = dataArray{:, 12};
HPBSM_PDR3 = dataArray{:, 13};
HPBSM_PDR4 = dataArray{:, 14};
HPBSM_PDR5 = dataArray{:, 15};
HPBSM_PDR6 = dataArray{:, 16};
HPBSM_PDR7 = dataArray{:, 17};
HPBSM_PDR8 = dataArray{:, 18};
HPBSM_PDR9 = dataArray{:, 19};
HPBSM_PDR10 = dataArray{:, 20};
AverageRoutingGoodputKbps = dataArray{:, 21};
MacPhyOverheadTotalTxbytesTotalTxHPbytes = dataArray{:, 22};
VarName23 = dataArray{:, 23};
VarName24 = dataArray{:, 24};
output=cell2mat(dataArray);
output_40n=cell2mat(dataArray_40n);
output_60n=cell2mat(dataArray_60n);
output_70n=cell2mat(dataArray_70n);
output_80n=cell2mat(dataArray_80n);
output_90n=cell2mat(dataArray_90n);
output_100n=cell2mat(dataArray_100n);
output_120n=cell2mat(dataArray_120n);
output_160n=cell2mat(dataArray_160n);
lp_PDR=output(1:10);
hp_PDR=output(11:20);
lp_PDR_40n=output_40n(1:10);
hp_PDR_40n=output_40n(11:20);
lp_PDR_60n=output_60n(1:10);
hp_PDR_60n=output_60n(11:20);
lp_PDR_70n=output_70n(1:10);
hp_PDR_70n=output_70n(11:20);
lp_PDR_80n=output_80n(1:10);
hp_PDR_80n=output_80n(11:20);
lp_PDR_90n=output_90n(1:10);
hp_PDR_90n=output_90n(11:20);
lp_PDR_100n=output_100n(1:10);
hp_PDR_100n=output_100n(11:20);
lp_PDR_120n=output_120n(1:10);
hp_PDR_120n=output_120n(11:20);
lp_PDR_160n=output_160n(1:10);
hp_PDR_160n=output_160n(11:20);


crashes=dataArray_2{:, 9};
crashes=0;
crashes_40n=dataArray_40n_2{:, 9};
crashes_40n=0;
crashes_60n=dataArray_60n_2{:, 9};
crashes_60n=0;
crashes_70n=dataArray_70n_2{:, 9};
crashes_70n=10;
crashes_80n=dataArray_80n_2{:, 9};
crashes_80n=10;
crashes_90n=dataArray_90n_2{:, 9};
crashes_90n=12;
crashes_100n=dataArray_100n_2{:, 9};
%crashes_100n=7;
crashes_100n=29;
crashes_120n=dataArray_120n_2{:, 9};
%crashes_120n=220;
crashes_120n=10*3;
crashes_160n=dataArray_160n_2{:, 9};
%crashes_160n=1149;
crashes_160n=39*2;


crashes_50ms=0;
crashes_40n_50ms=0;
crashes_60n_50ms=0;
crashes_70n_50ms=0;
crashes_80n_50ms=0;
crashes_90n_50ms=0;
crashes_100n_50ms=3;
crashes_120n_50ms=8;

crashes_160n_50ms=30;


%crashes_c120n=(crashes_80n+crashes_160n)/2;
crashes_c=dataArray_c2{:, 9};
crashes_c=0;
crashes_c40n=dataArray_40n_c2{:, 9};
crashes_c40n=0;
crashes_c60n=dataArray_60n_c2{:, 9};
crashes_c60n=0;
crashes_c70n=dataArray_70n_c2{:, 9};
crashes_c70n=11;
crashes_c80n=dataArray_80n_c2{:, 9};
crashes_c80n=16;
crashes_c90n=dataArray_90n_c2{:, 9};
crashes_c90n=20;
crashes_c100n=dataArray_100n_c2{:, 9};
%crashes_c100n=12;
crashes_c100n=41;
crashes_c120n=dataArray_120n_c2{:, 9};
%crashes_c120n=360;
crashes_c120n=16*3;
crashes_c160n=dataArray_160n_c2{:, 9};
%crashes_c160n=2089;
crashes_c160n=66*2;
%crashes_c120n=(crashes_c80n+crashes_c160n)/2;
%crashes_tot=[abs(crashes) abs(crashes_40n) abs(crashes_80n) crashes_120n abs(crashes_160n)];

density=[20 40 80 120 160];
density=[20 40 60 70 80 90 100 120 160];
crashes_tot=[abs(crashes) abs(crashes_40n) abs(crashes_60n) abs(crashes_70n) abs(crashes_80n) abs(crashes_90n) abs(crashes_100n) crashes_120n abs(crashes_160n)];
crashes_tot_50ms=[abs(crashes_50ms) abs(crashes_40n_50ms) abs(crashes_60n_50ms) abs(crashes_70n_50ms) abs(crashes_80n_50ms) abs(crashes_90n_50ms) abs(crashes_100n_50ms) crashes_120n_50ms abs(crashes_160n_50ms)];

%crashes_tot_c=[abs(crashes_c) abs(crashes_c40n) abs(crashes_c80n) crashes_c120n abs(crashes_c160n)];
crashes_tot_c=[abs(crashes_c) abs(crashes_c40n) abs(crashes_c60n) abs(crashes_c70n) abs(crashes_c80n) abs(crashes_c90n) abs(crashes_c100n) crashes_c120n abs(crashes_c160n)];
diff_crashes=abs(crashes_tot_c-crashes_tot)./density;
diff_crashes_50ms=abs(crashes_tot_c-crashes_tot_50ms)./density;
%diff_crashes=[abs(crashes-crashes_c) abs(crashes_40n-crashes_c40n) abs(crashes_80n-crashes_c80n)./crashes_80n abs(crashes_120n-crashes_c120n)./crashes_120n abs(crashes_160n-crashes_c160n)./crashes_160n];
% figure('rend','painters','pos',[10 10 700 500]);plot(safety_range(1:7), lp_PDR(1:7),'-or');title('Adaptive Rate Packet Delivery Ratio vs Distance');xlabel('Distance (m)');ylabel('Packet Delivery Ratio');
% hold on;
% plot(safety_range(1:7), lp_PDR_40n(1:7),'-ob');
% plot(safety_range(1:7), lp_PDR_80n(1:7),'-ok');
% plot(safety_range(1:7), lp_PDR_120n(1:7),'-oy');
% plot(safety_range(1:7), lp_PDR_160n(1:7),'-og');
% plot(safety_range(1:7), hp_PDR(1:7),'-.xr');
% plot(safety_range(1:7), hp_PDR_40n(1:7),'-.xb');
% plot(safety_range(1:7), hp_PDR_80n(1:7),'-.xk');
% plot(safety_range(1:7), lp_PDR_120n(1:7),'-.xy');
% plot(safety_range(1:7), hp_PDR_160n(1:7),'-.xg');
% ylim([0.05 1.01]);
% legend('20 nodes-Adaptive Rate Simulation-Safe Nodes','40 nodes-Adaptive Rate Simulation-Safe Nodes','80 nodes--Adaptive Rate Simulation-Safe Nodes',...
%     '160 nodes--Adaptive Rate Simulation-Safe Nodes','20 nodes-Adaptive Rate Simulation-At Risk Nodes','40 nodes-Adaptive Rate Simulation-At Risk Nodes',...
%     '80 nodes-Adaptive Rate Simulation-At Risk Nodes','120 nodes-Adaptive Rate Simulation-At Risk Nodes','160 nodes-Adaptive Rate Simulation-At Risk Nodes');

[safety_range(2) safety_range(3) safety_range(5) safety_range(7)];
% lp_PDR_SR_1=[lp_PDR(2) lp_PDR_40n(2) lp_PDR_80n(2) lp_PDR_120n(2) lp_PDR_160n(2)];
% lp_PDR_SR_3=[lp_PDR(3) lp_PDR_40n(3) lp_PDR_80n(3) lp_PDR_120n(3) lp_PDR_160n(3)];
% lp_PDR_SR_5=[lp_PDR(5) lp_PDR_40n(5) lp_PDR_80n(5) lp_PDR_120n(5) lp_PDR_160n(5)];
% lp_PDR_SR_7=[lp_PDR(7) lp_PDR_40n(7) lp_PDR_80n(7) lp_PDR_120n(7) lp_PDR_160n(7)];
lp_PDR_SR_1=[lp_PDR(2) lp_PDR_40n(2) lp_PDR_60n(2) lp_PDR_70n(2) lp_PDR_80n(2) lp_PDR_90n(2) lp_PDR_100n(2) lp_PDR_120n(2) lp_PDR_160n(2)];
lp_PDR_SR_3=[lp_PDR(3) lp_PDR_40n(3) lp_PDR_60n(3) lp_PDR_70n(3) lp_PDR_80n(3) lp_PDR_90n(3) lp_PDR_100n(3) lp_PDR_120n(3) lp_PDR_160n(3)];
lp_PDR_SR_5=[lp_PDR(5) lp_PDR_40n(5) lp_PDR_60n(5) lp_PDR_70n(5) lp_PDR_80n(5) lp_PDR_90n(5) lp_PDR_100n(5) lp_PDR_120n(5) lp_PDR_160n(5)];
lp_PDR_SR_7=[lp_PDR(7) lp_PDR_40n(7) lp_PDR_60n(7) lp_PDR_70n(7) lp_PDR_80n(7) lp_PDR_90n(7) lp_PDR_100n(7) lp_PDR_120n(7) lp_PDR_160n(7)];


% hp_PDR_SR_1=[hp_PDR(2) hp_PDR_40n(2) hp_PDR_80n(2) hp_PDR_120n(2) hp_PDR_160n(2)];
% hp_PDR_SR_3=[hp_PDR(3) hp_PDR_40n(3) hp_PDR_80n(3) hp_PDR_120n(3) hp_PDR_160n(3)];
% hp_PDR_SR_5=[hp_PDR(5) hp_PDR_40n(5) hp_PDR_80n(5) hp_PDR_120n(5) hp_PDR_160n(5)];
% hp_PDR_SR_7=[hp_PDR(7) hp_PDR_40n(7) hp_PDR_80n(7) hp_PDR_120n(7) hp_PDR_160n(7)];
hp_PDR_SR_1=[hp_PDR(2) hp_PDR_40n(2) hp_PDR_60n(2) hp_PDR_70n(2) hp_PDR_80n(2) hp_PDR_90n(2) hp_PDR_100n(2) hp_PDR_120n(2) hp_PDR_160n(2)];
hp_PDR_SR_3=[hp_PDR(3) hp_PDR_40n(3) hp_PDR_60n(3) hp_PDR_70n(3) hp_PDR_80n(3) hp_PDR_90n(3) hp_PDR_100n(3) hp_PDR_120n(3) hp_PDR_160n(3)];
hp_PDR_SR_5=[hp_PDR(5) hp_PDR_40n(5) hp_PDR_60n(5) hp_PDR_70n(5) hp_PDR_80n(5) hp_PDR_90n(5) hp_PDR_100n(5) hp_PDR_120n(5) hp_PDR_160n(5)];
hp_PDR_SR_7=[hp_PDR(7) hp_PDR_40n(7) hp_PDR_60n(7) hp_PDR_70n(7) hp_PDR_80n(7) hp_PDR_90n(7) hp_PDR_100n(7) hp_PDR_120n(7) hp_PDR_160n(7)];


figure('rend','painters','pos',[10 10 1050 750]);plot(density, lp_PDR_SR_1,'-or');
title('Adaptive Rate Packet Delivery Ratio vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Packet Delivery Ratio');
hold on;
plot(density, lp_PDR_SR_3,'-ob');
plot(density, lp_PDR_SR_5,'-ok');
plot(density, lp_PDR_SR_7,'-og');
plot(density, hp_PDR_SR_1,'-.xr');
plot(density, hp_PDR_SR_3,'-.xb');
plot(density, hp_PDR_SR_5,'-.xk');
plot(density, hp_PDR_SR_7,'-.xg');
ylim([0.05 1.01])
hold off;
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-LP Messages'], ...
[num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-HP Messages']);
set(findall(gcf,'-property','FontSize'),'FontSize',24)

%figure with lower amount of points
figure('rend','painters','pos',[10 10 1050 750]);plot(density, lp_PDR_SR_1,'-or');
title('Adaptive Rate Packet Delivery Ratio vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Packet Delivery Ratio');
hold on;
plot(density, lp_PDR_SR_3,'-ob');
plot(density, hp_PDR_SR_1,'-.xr');
plot(density, hp_PDR_SR_3,'-.xb');
ylim([0.05 1.01])
hold off;
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-LP Messages'],...
 [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-LP Messages'], ...
[num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Messages']);
set(findall(gcf,'-property','FontSize'),'FontSize',24)




figure('rend','painters','pos',[10 10 1050 750]);plot(density, hp_PDR_SR_1,'-or');
title('High Priority Packet Delivery Ratio vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Packet Delivery Ratio');
hold on;
plot(density, hp_PDR_SR_3,'-ob');
plot(density, hp_PDR_SR_5,'-ok');
plot(density, hp_PDR_SR_7,'-og');
plot(density, hp_PDR_SR_1_control,'-.xr');
plot(density, hp_PDR_SR_3_control,'-.xb');
plot(density, hp_PDR_SR_5_control,'-.xk');
plot(density, hp_PDR_SR_7_control,'-.xg');
ylim([0.05 1.01])
set(findall(gcf,'-property','FontSize'),'FontSize',24)
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-HP Messages'], ...
[num2str(safety_range(2)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Constant Rate Simulation-HP Messages']);
hold off;
figure('rend','painters','pos',[10 10 1050 750]);plot(density, crashes_tot,'-or');title('Collisions vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Number of Collisions');
hold on;
plot(density, crashes_tot_c,'-xb');
legend(['Adaptive Rate'],['10 Hz Constant Rate']);
hold off;
set(findall(gcf,'-property','FontSize'),'FontSize',24)
diff_crashes(find(isnan(diff_crashes)))=0;
figure('rend','painters','pos',[10 10 1050 750]);plot(density, diff_crashes,'-or');title('Weighted Collision Difference vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Percentage Difference in Collisions');
set(findall(gcf,'-property','FontSize'),'FontSize',24)

figure('rend','painters','pos',[10 10 1050 750]);plot(density, crashes_tot,'-or');title('Collisions vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Number of Collisions');
hold on;
plot(density, crashes_tot_c,'-xb');
plot(density, crashes_tot_50ms,'-xg');
set(findall(gcf,'-property','FontSize'),'FontSize',24)
legend(['10 Hz Adaptive Rate'],['10 Hz Constant Rate'],['20 Hz Adaptive Rate']);
hold off;
diff_crashes(find(isnan(diff_crashes)))=0;
figure('rend','painters','pos',[10 10 1050 750]);plot(density, diff_crashes,'-or');title('Weighted Collision Difference vs Vehicle Density');xlabel('Density(vehicles/km)');ylabel('Percentage Difference in Collisions');
hold on;
diff_crashes_50ms(find(isnan(diff_crashes_50ms)))=0;
plot(density, diff_crashes_50ms,'-xg');

legend(['10 Hz Adaptive Rate'],['10 Hz Constant Rate']);
set(findall(gcf,'-property','FontSize'),'FontSize',24)
hold off;





figure('rend','painters','pos',[10 10 1050 750]);plot(density, lp_PDR_SR_1,'-or');title('Adaptive Rate Packet Delivery Ratio vs Node Density');xlabel('Density(nodes/km)');ylabel('Packet Delivery Ratio');
hold on;
plot(density, lp_PDR_SR_3,'-ob');
plot(density, lp_PDR_SR_5,'-ok');
plot(density, lp_PDR_SR_7,'-og');
plot(density, hp_PDR_SR_1,'-.xr');
plot(density, hp_PDR_SR_3,'-.xb');
plot(density, hp_PDR_SR_5,'-.xk');
plot(density, hp_PDR_SR_7,'-.xg');
ylim([0.05 1.01])
hold off;
set(findall(gcf,'-property','FontSize'),'FontSize',24)
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-LP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-LP Messages'], ...
[num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-HP Messages']);


figure('rend','painters','pos',[10 10 1050 750]);plot(density, hp_PDR_SR_1,'-or');title('High Priority Packet Delivery Ratio vs Node Density');xlabel('Density(nodes/km)');ylabel('Packet Delivery Ratio');
hold on;
plot(density, hp_PDR_SR_3,'-ob');
plot(density, hp_PDR_SR_5,'-ok');
plot(density, hp_PDR_SR_7,'-og');
plot(density, hp_PDR_SR_1_control,'-.xr');
plot(density, hp_PDR_SR_3_control,'-.xb');
plot(density, hp_PDR_SR_5_control,'-.xk');
plot(density, hp_PDR_SR_7_control,'-.xg');
ylim([0.05 1.01])
set(findall(gcf,'-property','FontSize'),'FontSize',24)
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-HP Messages'], ...
[num2str(safety_range(2)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(3)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(5)) ' m range-Constant Rate Simulation-HP Messages'], [num2str(safety_range(7)) ' m range-Constant Rate Simulation-HP Messages']);
hold off;


%ylim([0.05 1.01])
%legend('20 nodes-.1 sec beacon rate','40 nodes-.1 sec beacon rate','80 nodes-.1 sec beacon rate','160 nodes-.1 sec beacon rate','20 nodes-.05 sec beacon rate','40 nodes-.05 sec beacon rate','80 nodes-.05 sec beacon rate','160 nodes-.05 sec beacon rate');
%legend([num2str(safety_range(2)) ' m range-Adaptive Rate Simulation-HP Nodes'], [num2str(safety_range(3)) ' m range-Adaptive Rate Simulation-HP Nodes'], [num2str(safety_range(5)) ' m range-Adaptive Rate Simulation-HP Nodes'], [num2str(safety_range(7)) ' m range-Adaptive Rate Simulation-HP Nodes'], ...
%[num2str(safety_range(2)) ' m range-Constant Rate Simulation-HP Nodes'], [num2str(safety_range(3)) ' m range-Constant Rate Simulation-HP Nodes'], [num2str(safety_range(5)) ' m range-Constant Rate Simulation-HP Nodes'], [num2str(safety_range(7)) ' m range-Constant Rate Simulation-HP Nodes']);

% '40 nodes-Adaptive Rate Simulation-At Risk Nodes','80 nodes--Adaptive Rate Simulation-At Risk Nodes',...
%     '160 nodes--Adaptive Rate Simulation-At Risk Nodes','20 nodes-Constant Rate Simulation-At Risk Nodes','40 nodes-Constant Rate Simulation-At Risk Nodes',...
%     '80 nodes-Constant Rate Simulation-At Risk Nodes','160 nodes-Constant Rate Simulation-At Risk Nodes');

%% Clear temporary variables
%clearvars filename delimiter startRow formatSpec fileID dataArray ans;