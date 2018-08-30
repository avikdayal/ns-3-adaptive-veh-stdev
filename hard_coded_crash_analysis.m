%crashes=dataArray_2{:, 9};
crashes=0;
%crashes_40n=dataArray_40n_2{:, 9};
crashes_40n=0;
%crashes_80n=dataArray_80n_2{:, 9};
crashes_80n=34;
%crashes_120n=dataArray_120n_2{:, 9};
crashes_120n=220;
%crashes_160n=dataArray_160n_2{:, 9};
crashes_160n=1149;
%crashes_c120n=(crashes_80n+crashes_160n)/2;
%crashes_c=dataArray_c2{:, 9};
crashes_c=0;
%crashes_c40n=dataArray_40n_c2{:, 9};
crashes_c40n=0;
%crashes_c80n=dataArray_80n_c2{:, 9};
crashes_c80n=40;
%crashes_c120n=dataArray_120n_c2{:, 9};
crashes_c120n=360;
%crashes_c160n=dataArray_160n_c2{:, 9};
crashes_c160n=2089;
%crashes_c120n=(crashes_c80n+crashes_c160n)/2;
crashes_tot=[abs(crashes) abs(crashes_40n) abs(crashes_80n) crashes_120n abs(crashes_160n)];

crashes_tot_c=[abs(crashes_c) abs(crashes_c40n) abs(crashes_c80n) crashes_c120n abs(crashes_c160n)];

diff_crashes=[abs(crashes-crashes_c) abs(crashes_40n-crashes_c40n) abs(crashes_80n-crashes_c80n)./crashes_c80n abs(crashes_120n-crashes_c120n)./crashes_c120n abs(crashes_160n-crashes_c160n)./crashes_c160n];

density=[20 40 80 120 160];
figure('rend','painters','pos',[10 10 700 500]);plot(density, crashes_tot,'-or');title('Crashes vs Node Density');xlabel('Density(nodes/km)');ylabel('Number of Crashes');
hold on;
plot(density, crashes_tot_c,'-xb');
legend(['Adaptive Scheme'],['Control Scheme']);
hold off;

figure('rend','painters','pos',[10 10 700 500]);plot(density, diff_crashes,'-or');title('Crash Difference Percentage vs Node Density');xlabel('Density(nodes/km)');ylabel('Number of Crashes');
