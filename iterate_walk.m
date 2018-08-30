filename = 'walk0.csv';
%M = csvread(filename,'\n',',')
%M = xlsread(filename)
fid = fopen('walk0.csv');
out = textscan(fid, ',');
fclose(fid)