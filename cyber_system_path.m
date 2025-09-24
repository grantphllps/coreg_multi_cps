close all;
clear;
clc;

%CPS1 components

inertia = 6.9137;
damping = 50;
control_input_range = [-15000, 15000];
lower_state_bounds = [-inf; 0.5];
upper_state_bounds = [inf; 20];
sampling_period = 1/10;
initial_state = [0; 5];

cyber_system1 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

mass = 1;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 2;
initial_state = [5; 0];
rates = 0.5:1:20;

physical_system1 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);

%Inverted pendulum is x dx theta dtheta

ps1_initial_state = [1; 0; 0.3; 0];
physical_system11 = inverted_pendulum(sampling_period,rates,ps1_initial_state);

%CPS2 components

inertia = 6.9137;
damping = 50;
control_input_range = [-15000, 15000];
lower_state_bounds = [-inf; 0.25];
upper_state_bounds = [inf; 20];
sampling_period = 1/10;
initial_state = [0; 5];

cyber_system2 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

m = 1;
k = 632;
b = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [0; 0];
rates = 0.25:0.25:20;

ps2_initial_state = [-1; 0; -0.3; 0];
physical_system2 = inverted_pendulum(sampling_period,rates,ps2_initial_state);

sub_2 = basic_coreg(physical_system2, cyber_system2);
sub_1 = basic_coreg(physical_system11, cyber_system1);

cps = multi_cps();
cps.add_sub_system(sub_1);
cps.add_sub_system(sub_2);

cps.set_cyber_system_trajectory({[5,5],[10,3],[1,15]})

%% Run the simulation

[trajectory] = cps.simulate([0,1]);

%% Plotting

ts_coreg = trajectory(:,end);

% Physical System 1
xp1_1 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(1)); %position
xp1_2 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(2));
xp1_3 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(3)); %theta
xp1_4 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(4));

% Physical System 2
xp2_1 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(1)); %position
xp2_2 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(2));

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

subplot(2,2,1)
hold on;
plot(ts_coreg,xp1_1);
plot(ts_coreg,xp1_3);
plot(ts_coreg,xp1_u)
plot(up1_ts,-2,'|',"color",'green')
title("Physical System 1")
legend("Translational Position","rotational position","Control Input")
hold off;

subplot(2,2,2);
hold on;
plot(ts_coreg,xp2_1);
%plot(ts_coreg,xp2_2);
plot(ts_coreg,xp2_u)
plot(up2_ts,-0.35,'|',"color",'green')
%plot(up2_ts,up2_us,'x')
title("Physical System 2")
legend("Translational Position","Control Input")
hold off;

subplot(2,2,3)
hold on;
plot(ts_coreg,xc1_2)
%plot(ts_coreg,xc1_u)
title("Cyber System 1")
legend("Rotational Velocity (rads/sec)", "Cyber Control Input")
hold off

subplot(2,2,4)
hold on;
plot(ts_coreg,xc2_2)
%plot(ts_coreg,xc2_u)
title("Cyber System 2")
legend("Rotational Velocity (rads/sec)", "Cyber Control Input")
hold off
