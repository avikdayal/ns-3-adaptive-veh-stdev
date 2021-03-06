%% Import data from text file.
% Script for importing data from the following text file:
%
%    /Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-RA-simulation/ns-3-dev/walk0.unsorted.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2018/04/08 21:52:13

%% Initialize variables.
filename = '/Users/avikdayal/Google-Drive/PHD-Thesis/ARL-work/ns-3-RA-simulation/ns-3-dev/walk0.unsorted.csv';
delimiter = ',';
startRow = 16;

%% Format for each line of text:
%   column1: text (%s)
%	column2: text (%s)
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
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
time_ms = dataArray{:, 1};
S = dataArray{:, 2};
node_num = dataArray{:, 3};
packet_id = dataArray{:, 4};
priority = dataArray{:, 5};
distance = dataArray{:, 7};
%VarName7 = dataArray{:, 7};
velocity = dataArray{:, 6};
num_p_sent = dataArray{:, 8};
VarName9 = dataArray{:, 9};
VarName10 = dataArray{:, 10};
VarName11 = dataArray{:, 11};
VarName12 = dataArray{:, 12};
VarName13 = dataArray{:, 13};
VarName14 = dataArray{:, 14};
VarName15 = dataArray{:, 15};
VarName16 = dataArray{:, 16};
VarName17 = dataArray{:, 17};

pdr_stats_nodes_sent=zeros(1,max(packet_id));
pdr_stats_nodes_rec=zeros(1,max(packet_id));

packet_id_start=packet_id(time_offset);
packet_id_end=packet_id(length(time_ms)-time_offset);

pdr_stats_nodes_sent_hp=zeros(1,max(packet_id));
pdr_stats_nodes_rec_hp=zeros(1,max(packet_id));

priority_tracker=zeros(1,max(packet_id));
packet_id_tracker=1;
%10000 to length-100000 to give time for sim to settle
time_offset=10000;

t_velocity=zeros(1,max(packet_id));
for i=time_offset:length(time_ms)-time_offset
    
    if( strcmp(S(i),'S'))
        pdr_stats_nodes_sent(packet_id(i))=num_p_sent(i);
        priority_tracker(packet_id(i))=priority(i);
        t_velocity(packet_id(i))=velocity(i);
    end
    if(strcmp(S(i),'R'))
        pdr_stats_nodes_rec(packet_id(i))=pdr_stats_nodes_rec(packet_id(i))+1;
        
    end
    
end

pdr_stats_nodes=pdr_stats_nodes_rec./pdr_stats_nodes_sent;
%packets_to_remove=[1 2 length(pdr_stats_nodes)];
packets_to_remove=[];
pdr_stats_nodes(packets_to_remove)=[];
priority_tracker(packets_to_remove)=[];
avg_pdr_by_priority=[];
priority_levels = unique(priority);
priority_levels(isnan(priority_levels)) = [];
velocity(isnan(velocity))=[];
velocity(packets_to_remove)=[];
for j=1:length(priority_levels)
    avg_pdr_by_priority=[avg_pdr_by_priority mean(pdr_stats_nodes(find(priority_tracker==priority_levels(j))))];
end   



%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;