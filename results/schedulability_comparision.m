clc;
close all;
clear;

fontsize = 16
lineweight = 1
marker =  "."
markerSize = 16

n2 = load('3x2diff.mat');
n3 = load('3x3diff.mat');
n4 = load('3x4diff.mat');
n5 = load('3x5diff.mat');
n6 = load('3x6diff.mat');

utilization_values = 0.05:0.05:0.95;
utilization_values = utilization_values';

fig = figure;
hold on;

plot(utilization_values, n2.diff,"-o","LineWidth",lineweight,"Marker",marker,"MarkerSize",markerSize)
plot(utilization_values, n3.diff,"-o","LineWidth",lineweight,"Marker",marker,"MarkerSize",markerSize)
plot(utilization_values, n4.diff,"-o","LineWidth",lineweight,"Marker",marker,"MarkerSize",markerSize)
plot(utilization_values, n5.diff,"-o","LineWidth",lineweight,"Marker",marker,"MarkerSize",markerSize)
plot(utilization_values, n6.diff,"-o","LineWidth",lineweight,"Marker",marker,"MarkerSize",markerSize);

title("Dependent Taskset Schedulability Improvement","FontSize",fontsize,"FontName","Times")
legend(["2 Modes","3 Modes","4 Modes", "5 Modes", "6 Modes"])
xlabel("Utilization","FontSize",fontsize,"FontName","Times")
ylabel("Improvement","FontSize",fontsize,"FontName","Times")

xlim([0.05, 1])
ylim([0, 0.22])

xticks(0.05:0.05:0.95)
yticks(0.02:0.02:.22)

grid on

ax = gca;
ax.FontSize = fontsize;





