%Plotting

close all;

ts_coreg = trajectory(:,end);

% Physical System 1
xp1_1 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(1)); %position
xp1_2 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(2));
xp1_3 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(3)); %theta
xp1_4 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(4));

% Physical System 2
xp2_1 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(1)); %position
xp2_2 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(2));
xp2_3 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(3)); %theta
xp1_4 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(4));

[~, things] = size(trajectory);

xp1_u = trajectory(:,things-4);
xc1_u = trajectory(:,things-3);
xp2_u = trajectory(:,things-2);
xc2_u = trajectory(:,things-1);

xc1_2 = trajectory(:,cps.sub_systems{1}.cps_xcidcs(2));
xc2_2 = trajectory(:,cps.sub_systems{2}.cps_xcidcs(2));

up1_ts = cps.sub_systems{1}.physical_system.input_updates(1,:);
up1_us = cps.sub_systems{1}.physical_system.input_updates(2,:);

up2_ts = cps.sub_systems{2}.physical_system.input_updates(1,:);
up2_us = cps.sub_systems{2}.physical_system.input_updates(2,:);

figure;

fontsize = 16;
lineweight = 1;
marker =  ".";
markerSize = 16;

pxlims = 8;
psxlabel = "Time (s)";
psylabel1 = "Position (meters)";
psylabel2 = "Angle (rad)";
psylims1 = [-1 1];
psylims2 = [-.4 .4];

csylims = [0 10];

subplot(2,2,1)
hold on;

yyaxis left
plot(ts_coreg,xp1_1,"LineWidth",lineweight);
ylabel(psylabel1,"FontSize",fontsize,"FontName","Times","Color","Black")
ylim(psylims1)
yticks(-1:0.2:1)

yyaxis right
plot(ts_coreg,xp1_3,"--","LineWidth",lineweight);
ylabel(psylabel2,"FontSize",fontsize,"FontName","Times","Color","Black")
ylim(psylims2)

xlabel(psxlabel,"FontSize",fontsize)
% plot(ts_coreg,xp1_u)
plot(up1_ts,0,'.',"color",'r',"MarkerSize",10)
title("Physical System 1","FontSize",fontsize,"FontName","Times")
legend("Cart Position","Pendulum Position")
xlim([0 8])

xticks(0:0.5:8)
yticks(-1:0.1:1)

grid on;
ax = gca;
ax.FontSize = fontsize;
ax.FontName = "Times";
hold off;

%End first

subplot(2,2,2);
hold on;
yyaxis left
plot(ts_coreg,xp2_1,"LineWidth",lineweight);
ylabel(psylabel1,"FontSize",fontsize,"FontName","Times","Color","Black")
ylim(psylims1)
yticks(-1:0.2:1)

yyaxis right
plot(ts_coreg,xp2_3,"--","LineWidth",lineweight);
ylabel(psylabel2,"FontSize",fontsize,"FontName","Times","Color","Black")
ylim(psylims2)

xlabel(psxlabel,"FontSize",fontsize,"FontName","Times")
% plot(ts_coreg,xp1_u)
plot(up2_ts,0,'.',"color",'r',"MarkerSize",10)
title("Physical System 2","FontSize",fontsize,"FontName","Times","Color","Black")
legend("Cart Position","Pendulum Position")
xlim([0 8])

xticks(0:0.5:8)
yticks(-1:0.1:1)

grid on;
ax = gca;
ax.FontSize = fontsize;
ax.FontName = "Times";
hold off;

%End second

subplot(2,2,3)
hold on;
plot(ts_coreg,xc1_2,"LineWidth",lineweight)
%plot(ts_coreg,xc1_u)
title("Cyber System 1","FontSize",fontsize,"FontName","Times")
xlabel("Time (s)","FontSize",fontsize,"FontName","Times")
ylabel("Sampling Rate (Hz)","FontSize",fontsize,"FontName","Times")
ylim(csylims)

legend("Control Job Release Rate")

xticks(0:0.5:8)
yticks(0:1:10)

grid on;
ax = gca;
ax.FontSize = fontsize;
ax.FontName = "Times"
xlim([0 8])
hold off;

%End thrid

subplot(2,2,4)
hold on;
plot(ts_coreg,xc2_2,"LineWidth",lineweight)
%plot(ts_coreg,xc2_u)
title("Cyber System 2","FontSize",fontsize,"FontName","Times")
xlabel("Time (s)","FontSize",fontsize,"FontName","Times")
ylabel("Sampling Rate (Hz)","FontSize",fontsize,"FontName","Times")
ylim(csylims)

legend("Control Job Release Rate")

xticks(0:0.5:8)
yticks(0:1:10)

grid on;
ax = gca;
xlim([0 8])
ax.FontSize = fontsize;
ax.FontName = "Times"
hold off

%End fourth
