clc;
clear;
close all;


% All times in milliseconds
rates = 1:1:10;
Ts = rates.^-1;
Ts = sort(Ts)';
Ts = Ts*1000;
Cs = 5 * ones(10,1);
modes1 = [Ts Cs];

%modes1 = [9.23, .246; 10.91, .2777];  % Nx2 matrix a
%periodic_task = PeriodicTask(2.0, 20.0, 20.0, 1);
avr_task1 = AVRTask(modes1, 1000.0, -1000, 1000, 60000.0, 1);

%modes2 = [20, 0.8; 25, .6];  % Nx2 matrix
%periodic_task = PeriodicTask(2.0, 20.0, 20.0, 1);
avr_task2 = AVRTask(modes1, 1000.0, -1000, 1000, 60000.0, 2);

task_set = [avr_task1, avr_task2];

schedulable = is_task_set_schedulable(task_set);
disp(schedulable);

%%

is = [];
as = [];
w = 40; %ms

figure;
hold on;

    idx = 1;
    is = [];
    as = [];
    for i = 15:0.1:20
        avr_task = AVRTask(modes1,w,-1*i,i);
        is(idx) = avr_task.interference(w);
        as(idx) = i;
        idx = idx + 1;
    end
    plot(as,is,'x')

