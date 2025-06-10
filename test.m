close all;
clear;
clc;

inertia = 2.0;
damping = 1;
control_input_range = [-20,20];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 20.1];

cyber_system = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);
% bar = cyber_system.simulate([0,10]);

mass = 0.5;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [3; 10];
rates = 1:1:20;

physical_system = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);
% bar = foo.simulate([0,10]);

cps = basic_coreg(physical_system, cyber_system);

results = cps.simulate([0,10]);

ts = results(:,end);
xp1 = results(:,1);
xp2 = results(:,2);
xc1 = results(:,3);
xc2 = results(:,4);
up = results(:,5);
uc = results(:,6);

figure; hold on;

plot(ts,xp1);
plot(ts,xp2);
% plot(ts,xc1);
plot(ts,xc2);
plot(ts,up);
plot(ts,uc);

legend(["xp1","xp2","xc2","up","uc"])

% figure;
% hold on;
% plot(bar(:,end), bar(:,1)) %Position
% plot(bar(:,end), bar(:,2)) %Velocity
% plot(bar(:,end), bar(:,3)) %Control Input
% legend(["Position", "Velocity", "Control Input"])
