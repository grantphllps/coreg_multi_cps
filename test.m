close all;
clear;
clc;

%CPS1 components

inertia = 10;
damping = 1;
control_input_range = [-40,40];
lower_state_bounds = [-inf; 0.5];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 2];

cyber_system1 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

mass = 1;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 2;
initial_state = [.2; 0];
rates = 0.5:1:20;

physical_system1 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);

%Inverted pendulum is x dx theta dtheta

ps1_initial_state = [1; 0; 0.3; 0];
physical_system11 = inverted_pendulum(sampling_period,rates,ps1_initial_state);

%CPS2 components

inertia = 20;
damping = 1;
control_input_range = [-40,40];
lower_state_bounds = [-inf; 0.25];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 2];

cyber_system2 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

% mass = 1;
% damping = 1;
% Q = eye(2);
% R = 1;
% sampling_period = 1;
% initial_state = [4; -10];
% rates = 1:1:20;
% 
% physical_system2 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);

m = 1;
k = 632;
b = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [0; 0];
rates = 0.25:0.25:20;
physical_system2 = spring_mass_damper(m,k,b,Q,R,sampling_period,rates,initial_state);


%CPS3 components

inertia = 10;
damping = 1;
control_input_range = [-20,20];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 2];

cyber_system3 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

mass = 0.5;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [-12; -2];
rates = 1:1:20;

physical_system3 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);


% Disturbances
dist1 = wave(1/10, 5);
dist2 = constant(1/10);

sub_3 = basic_coreg(physical_system3, cyber_system3);
sub_2 = basic_coreg(physical_system2, cyber_system2);
sub_1 = basic_coreg(physical_system11, cyber_system1);

cps = multi_cps();
cps.add_sub_system(sub_1);
cps.add_sub_system(sub_2);
cps.add_sub_system(sub_3);
cps.add_disturbance(dist1,1);
cps.add_disturbance(dist2,3);

%% Unit tests
% physical_system1 = cps.sub_systems{1}.physical_system.A;
% if any(any(physical_system1 ~= cps.A(1:2,1:2)))
%     error("Faild CPS dimension check")
% end
% 
% cyber_system1 = cps.sub_systems{1}.cyber_system.A;
% if any(any(cyber_system1 ~= cps.A(3:4,3:4)))
%     error("Faild CPS dimension check")
% end
% 
% physical_system1 = cps.sub_systems{2}.physical_system.A;
% if any(any(physical_system1 ~= cps.A(5:6,5:6)))
%     error("Faild CPS dimension check")
% end
% 
% cyber_system1 = cps.sub_systems{2}.cyber_system.A;
% if any(any(cyber_system1 ~= cps.A(7:8,7:8)))
%     error("Faild CPS dimension check")
% end
% 
% psb = cps.sub_systems{1}.physical_system.B;
% if any(psb ~= cps.B(1:2,1))
%     error("Faild CPS dimension check")
% end
% 
% disp("All dimension tests passed")

%sub_1.simulate([0, 10])

%% Simulate

[trajectory] = cps.simulate([0,15]);

%% Plottings

ts_coreg = trajectory(:,end);

% Physical System 1
xp1_1 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(1)); %position
xp1_2 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(2));
xp1_3 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(3)); %theta
xp1_4 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(4));

% Physical System 2
xp2_1 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(1)); %position
xp2_2 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(2));

% Physical System 3
xp3_1 = trajectory(:,cps.sub_systems{3}.cps_xpidcs(1)); %position
xp3_2 = trajectory(:,cps.sub_systems{3}.cps_xpidcs(2));


[~, things] = size(trajectory);

xp1_u = trajectory(:,things-6);
xc1_u = trajectory(:,things-5);
xp2_u = trajectory(:,things-4);
xc2_u = trajectory(:,things-3);
xp3_u = trajectory(:,things-2);
xc3_u = trajectory(:,things-1);

xc1_2 = trajectory(:,cps.sub_systems{1}.cps_xcidcs(2));
xc2_2 = trajectory(:,cps.sub_systems{2}.cps_xcidcs(2));
xc3_2 = trajectory(:,cps.sub_systems{3}.cps_xcidcs(2));

up1_ts = cps.sub_systems{1}.physical_system.input_updates(1,:);
up1_us = cps.sub_systems{1}.physical_system.input_updates(2,:);

up2_ts = cps.sub_systems{2}.physical_system.input_updates(1,:);
up2_us = cps.sub_systems{2}.physical_system.input_updates(2,:);

up3_ts = cps.sub_systems{3}.physical_system.input_updates(1,:);
up3_us = cps.sub_systems{3}.physical_system.input_updates(2,:);

figure;

subplot(2,3,1);
hold on;
plot(ts_coreg,xp1_1);
%plot(ts_coreg,xp1_2);
plot(ts_coreg,xp1_3);
%plot(ts_coreg,xp1_4)
plot(ts_coreg,xp1_u)
plot(up1_ts,-2,'|',"color",'green')
%plot(up1_ts,up1_us,'x')
title("Physical System 1")
legend("Translational Position","rotational position","Control Input")
hold off;

subplot(2,3,2);
hold on;
plot(ts_coreg,xp2_1);
%plot(ts_coreg,xp2_2);
plot(ts_coreg,xp2_u)
plot(up2_ts,-0.35,'|',"color",'green')
%plot(up2_ts,up2_us,'x')
title("Physical System 2")
legend("Translational Position","Control Input")
hold off;

subplot(2,3,3);
hold on;
plot(ts_coreg,xp3_1);
plot(ts_coreg,xp3_2);
plot(ts_coreg,xp3_u)
%plot(up3_ts,up3_us,'x')
title("Physical System 2")
legend("Translational Position","Translational Velocity","Control Input")
hold off;

subplot(2,3,4)
hold on;
plot(ts_coreg,xc1_2)
plot(ts_coreg,xc1_u)
title("Cyber System 1")
legend("Rotational Velocity (rads/sec)", "Cyber Control Input")
hold off

subplot(2,3,5)
hold on;
plot(ts_coreg,xc2_2)
plot(ts_coreg,xc2_u)
title("Cyber System 2")
legend("Rotational Velocity (rads/sec)", "Cyber Control Input")
hold off

subplot(2,3,6)
hold on;
plot(ts_coreg,xc3_2)
plot(ts_coreg,xc3_u)
title("Cyber System 3")
legend("Rotational Velocity (rads/sec)", "Cyber Control Input")
hold off


figure; hold on;
plot(up1_ts,0,'|',"color",'green')
plot(up2_ts,.2,'|',"Color","black")
plot(up3_ts,.4,'|',"Color","blue")
title("Physical System Updates")
legend("Physical System 1","Physical System 2", "Physical System 3")
ylim([-3 3])
hold off;

%% Coreg CPS Analysis

rms_xp1_3 = compute_ase(ts_coreg,xp1_3,zeros(1,length(ts_coreg)));
%rms_xp1_3 = rms(xp1_3);

%% Single, Vanilla system

m = 1;
k = 632;
b = 1;
Q = eye(2);
R = 1;
sampling_period = 1/10;
initial_state = [1; 0; 0.3; 0];
rates = 0.25:0.25:20;

sampling_rates = 1:1:20;
figure;

idx = 1;
ases = [];

for i = sampling_rates

    sampling_period  = 1/i;
    subplot(2,10,i)
    hold on;
    
    physical_system_copy = inverted_pendulum(sampling_period,rates,ps1_initial_state);
    physical_system_copy.add_disturbance(dist1);
    trajectory_new = physical_system_copy.simulate([0,15]);
    
    ts = trajectory_new(:,6);
    us = trajectory_new(:,5);
    x4 = trajectory_new(:,4);
    x3 = trajectory_new(:,3);
    x2 = trajectory_new(:,2);
    x1 = trajectory_new(:,1);

    ases(idx) = compute_ase(ts,x3,zeros(1,length(ts)));
    % ases(idx) = rms(x3);
    idx = idx + 1;
    
    title([i,"Hz"])
    plot(ts,x3); %plots the vanilla response
    plot(ts_coreg,xp1_3); %plots the coreg response
    legend(["Fixed Rate Response","Co-regulated Response"])
    %plot(ts,us);

    ylim([-1.5,1.5])
    
    hold off;
end
%% Comparision
figure;
subplot(2,1,1)
hold on;
plot(sampling_rates(3:end), ases(3:end))
plot(10,rms_xp1_3,'x')
xlabel("Sampling Rate (Hz)")
ylabel("Average Squared Error")
hold off;

subplot(2,1,2)
hold on;
samples = 15*sampling_rates;
plot(sampling_rates,samples);
plot(10, length(up1_ts), 'x')
xlabel("Sampling Rate (Hz)")
ylabel("Number of Samples Used")
legend(["Fixed Rate","Co-regulation"])
hold off;




%%

function ase = compute_ase(ts, xs, xs_ref)
    % compte the time average state error
    sum = 0;

    for i = 1:length(ts) - 1
        delta_t = ts(i+1) - ts(i);
        error = xs(i) - xs_ref(i);
        error = error^2;
        sum = sum + error*delta_t;
    end
    
    t_total = ts(end) - ts(1);
   ase = sum / t_total;
end