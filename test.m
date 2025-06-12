close all;
clear;
clc;

%CPS1 components

inertia = 1;
damping = 1;
control_input_range = [-20,20];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 10];

cyber_system1 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

mass = 1;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [7; 0];
rates = 1:1:20;

physical_system1 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);

%CPS2 components

inertia = .3;
damping = 1;
control_input_range = [-20,20];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 20];
sampling_period = 1/4;
initial_state = [0; 5];

cyber_system2 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state);

mass = 0.5;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 1;
initial_state = [3; -5];
rates = 1:1:20;

physical_system2 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);


sub_2 = basic_coreg(physical_system2, cyber_system2);
sub_1 = basic_coreg(physical_system1, cyber_system1);

cps = multi_cps();
cps.add_sub_system(sub_1);
cps.add_sub_system(sub_2);

% ts = results(:,end);
% xp1 = results(:,1);
% xp2 = results(:,2);
% xc1 = results(:,3);
% xc2 = results(:,4);
% up = results(:,5);
% uc = results(:,6);

% figure; hold on;

% plot(ts,xp1);
% plot(ts,xp2);
% plot(ts,xc1);
% plot(ts,xc2);
% plot(ts,up);
% plot(ts,uc);

% legend(["xp1","xp2","xc2","up","uc"])

% figure;
% hold on;
% plot(bar(:,end), bar(:,1)) %Position
% plot(bar(:,end), bar(:,2)) %Velocity
% plot(bar(:,end), bar(:,3)) %Control Input
% legend(["Position", "Velocity", "Control Input"])

%Unit tests
physical_system1 = cps.sub_systems{1}.physical_system.A;
if any(any(physical_system1 ~= cps.A(1:2,1:2)))
    error("Faild CPS dimension check")
end

cyber_system1 = cps.sub_systems{1}.cyber_system.A;
if any(any(cyber_system1 ~= cps.A(3:4,3:4)))
    error("Faild CPS dimension check")
end

physical_system1 = cps.sub_systems{2}.physical_system.A;
if any(any(physical_system1 ~= cps.A(5:6,5:6)))
    error("Faild CPS dimension check")
end

cyber_system1 = cps.sub_systems{2}.cyber_system.A;
if any(any(cyber_system1 ~= cps.A(7:8,7:8)))
    error("Faild CPS dimension check")
end

psb = cps.sub_systems{1}.physical_system.B;
if any(psb ~= cps.B(1:2,1))
    error("Faild CPS dimension check")
end

disp("All dimension tests passed")

%sub_1.simulate([0, 10])

[trajectory, updates] = cps.simulate([0,10]);

ts = trajectory(:,end);
xp1_1 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(1));
xp1_2 = trajectory(:,cps.sub_systems{1}.cps_xpidcs(2));
xp2_1 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(1));
xp2_2 = trajectory(:,cps.sub_systems{2}.cps_xpidcs(2));
xp1_u = trajectory(:,9);
xc1_u = trajectory(:,10);
xp2_u = trajectory(:,11);
xc2_u = trajectory(:,12);

xc1_2 = trajectory(:,cps.sub_systems{1}.cps_xcidcs(2));
xc2_2 = trajectory(:,cps.sub_systems{2}.cps_xcidcs(2));



figure;

subplot(2,2,1);
hold on;
plot(ts,xp1_1);
plot(ts,xp1_2);
plot(ts,xp1_u)
plot(updates(1,:),-1*updates(2,:),'x')
legend("xp1","xp2","up","update")
hold off;

subplot(2,2,3)
hold on;
plot(ts,xc1_2)
plot(ts,xc1_u)
hold off;

subplot(2,2,2);
hold on;
plot(ts,xp2_1);
plot(ts,xp2_2);
plot(ts,xp2_u);
hold off;

subplot(2,2,4);
hold on;
plot(ts,xc2_2);
plot(ts,xc2_u);
hold off;
