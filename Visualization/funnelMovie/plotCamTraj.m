close all;
file_num = '2015_02_21_17';
load(['../../../Logs/Feb212015/lcmlog_' file_num '.mat'])
tt = Camera_x(:,1);
tt_start = tt(1);
[xtrajCam,xtrajSBach] = getCameraTraj(['../../../Logs/Feb212015/lcmlog_' file_num '.mat'],tt_start);
ts = xtrajCam.getBreaks();
xs = xtrajCam.eval(ts);
for k = 1:6
figure
plot(ts,xs(k,:));
xlim([35 55]);
% ylim([-2 2])
end