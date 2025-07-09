close all;
clear;
clc;

%% COMPARISON PARAMETERS
SIM_START = 0;
SIM_END = 5;
SIM_CONTROL_RATES = 1:1:20;
SIM_DISTURBANCE_RATES = [1,2,3,4,5];
SIM_SPAN = [SIM_START, SIM_END];

REFERENCE_TRAJECTORY = [3 1 0 0 0; 6 1 0 0 0];

% CS1 components
cs1_inertia = 2;
cs1_damping = 1;
cs1_control_input_range = [-40, 40];
cs1_lower_state_bounds = [-inf; SIM_CONTROL_RATES(1)];
cs1_upper_state_bounds = [inf; SIM_CONTROL_RATES(end)];
cs1_sampling_period = 1/4;
cs1_initial_state = [0; 8];

% PS1 Components
ps1_mass = 1;
ps1_damping = 1;
ps1_Q = eye(2);
ps1_R = 1;
ps1_sampling_period = 2; %doesn't matter for coreg
ps1_initial_state = [0; 0; 0.3; 0];
ps1_rates = SIM_CONTROL_RATES;

%% Compare non-disturbed responses

traj_array = {};
ase_array = [];
compute_array = [];

for i = 1:length(SIM_CONTROL_RATES)
    test_sampling_rate = SIM_CONTROL_RATES(i);
    test_sampling_period = 1 / test_sampling_rate;

    test_system = inverted_pendulum(test_sampling_period, SIM_CONTROL_RATES,ps1_initial_state);
    test_traj = test_system.simulate(SIM_SPAN);

    ts = test_traj(:,end);
    xp3 = test_traj(:,3);

    traj_array{i} = test_traj;
    ase_array(i) = compute_ase(ts,xp3,zeros(1,length(ts)));
    compute_array(i) = test_sampling_rate * SIM_END;
end

test_cps = multi_cps();
cyber_system = simple_rotational(cs1_inertia,cs1_damping,cs1_control_input_range,cs1_lower_state_bounds,cs1_upper_state_bounds,cs1_sampling_period,cs1_initial_state);
coreg_sys = basic_coreg(test_system, cyber_system);
test_cps.add_sub_system(coreg_sys);

test_traj = test_cps.simulate(SIM_SPAN);

ts = test_traj(:,end);
xp3 = test_traj(:,test_cps.sub_systems{1}.cps_xpidcs(3)); %theta
up1 = test_cps.sub_systems{1}.physical_system.input_updates(1,:);

traj_array{i+1} = test_traj;
ase_array(i+1) = compute_ase(ts, xp3, zeros(length(ts)));
compute_array(i+1) = length(up1);

figure;

for i = 1:length(traj_array)-1
    subplot(2,10,i)
    hold on;
    ts = traj_array{i}(:,end);
    theta = traj_array{i}(:,3);

    plot(ts,theta)

    ts = traj_array{end}(:,end);
    xp3 = traj_array{end}(:,3);
    plot(ts,xp3);
    xlabel("time")
    ylabel("angle (radians)")

    legend("Static", "Coregulated")
    title(["Comparion",i])
    ylim([-0.3,0.3])
    hold off;
end

figure
hold on;
for i = 1:length(ase_array)
    plot(compute_array(1:end-1),ase_array(1:end-1))
    plot(compute_array(end),ase_array(end),'x')
end

xlabel("control computes")
ylabel("average squared error")

%% Various Co-regulated cs initial conditions
traj_array = {};
ase_array = [];
compute_array = [];

figure;
hold on;

for i = SIM_CONTROL_RATES
    test_cps = multi_cps();
    cs1_initial_state = [0; i];
    cyber_system = simple_rotational(cs1_inertia,cs1_damping,cs1_control_input_range,cs1_lower_state_bounds,cs1_upper_state_bounds,cs1_sampling_period,cs1_initial_state);
    
    coreg_sys = basic_coreg(test_system, cyber_system);
    test_cps.add_sub_system(coreg_sys);
    
    test_traj = test_cps.simulate(SIM_SPAN);

    ts = test_traj(:,end);
    xp3 = test_traj(:,test_cps.sub_systems{1}.cps_xpidcs(3)); %theta
    up1 = test_cps.sub_systems{1}.physical_system.input_updates(1,:);
    
    traj_array{i} = test_traj;
    ase_array(i) = compute_ase(ts, xp3, zeros(length(ts)));
    compute_array(i) = length(up1);

end

plot(SIM_CONTROL_RATES,ase_array)

%% Trajectory/Reference Tracking Across Controller Designs
traj_array = {};
ase_array = [];
compute_array = [];

SIM_END = 10; %<-Re-define sim end
SIM_SPAN = [SIM_START, SIM_END];

for i = 1:length(SIM_CONTROL_RATES)
    test_sampling_rate = SIM_CONTROL_RATES(i);
    test_sampling_period = 1 / test_sampling_rate;

    test_system = inverted_pendulum(test_sampling_period, SIM_CONTROL_RATES,ps1_initial_state);
    test_system.add_reference(REFERENCE_TRAJECTORY) % <- Here is the reference
    test_traj = test_system.simulate(SIM_SPAN);

    ts = test_traj(:,end);
    xp1 = test_traj(:,1);

    traj_array{i} = test_traj;
    ase_array(i) = compute_ase(ts,xp1,zeros(1,length(ts)));
    compute_array(i) = test_sampling_rate * SIM_END;
end

test_cps = multi_cps();
cyber_system = simple_rotational(cs1_inertia,cs1_damping,cs1_control_input_range,cs1_lower_state_bounds,cs1_upper_state_bounds,cs1_sampling_period,cs1_initial_state);
test_system = inverted_pendulum(test_sampling_period, SIM_CONTROL_RATES,ps1_initial_state);
test_system.add_reference(REFERENCE_TRAJECTORY) % <- Here is the reference

coreg_sys = basic_coreg(test_system, cyber_system);
test_cps.add_sub_system(coreg_sys);

test_traj = test_cps.simulate(SIM_SPAN);

ts = test_traj(:,end);
xp1 = test_traj(:,test_cps.sub_systems{1}.cps_xpidcs(1)); %theta
up1 = test_cps.sub_systems{1}.physical_system.input_updates(1,:);

traj_array{i+1} = test_traj;
ase_array(i+1) = compute_ase(ts, xp1, zeros(length(ts)));
compute_array(i+1) = length(up1);

figure;

for i = 1:length(traj_array)-1
    subplot(2,10,i)
    hold on;
    ts = traj_array{i}(:,end);
    theta = traj_array{i}(:,1);

    plot(ts,theta)

    ts = traj_array{end}(:,end);
    xp1 = traj_array{end}(:,1);
    plot(ts,xp1);
    xlabel("time")
    ylabel("cart position meters")

    legend("Static", "Coregulated")
    title(["Comparion",i])
    ylim([-0.5,1.5])
    hold off;
end

figure
hold on;
for i = 1:length(ase_array)
    plot(compute_array(1:end-1),ase_array(1:end-1))
    plot(compute_array(end),ase_array(end),'x')
end

xlabel("control computes")
ylabel("average squared error")

figure;
subplot(1,2,1)
hold on;
ts =  traj_array{end}(:,end);
xp1 = traj_array{end}(:,1);
xp2 = traj_array{end}(:,2);
xp3 = traj_array{end}(:,3);
xp4 = traj_array{end}(:,4);
xc1 = traj_array{end}(:,5);
xc2 = traj_array{end}(:,6);
up1 = traj_array{end}(:,7);
uc1 = traj_array{end}(:,8);

plot(ts,xp1);
plot(ts,up1);
legend(["Position","Control Effort"])

xlabel("Time")
ylabel("Position from Origin")
title("Physical System Response")

hold off;

subplot(1,2,2)
hold on;
plot(ts,xc2);
plot(ts,uc1);
legend(["Angular Velocity","Control Effort"])

xlabel("Time")
ylabel("Angular Velocity")
title("Cyber System Response")


%% Comparision Simulations
traj_array = {}; %[control_rate, disturbance_rate]
ase_array = [];
compute_array = []

test_control_rates = 1:1:20;
test_distrubance_rates = [1,2,3,4,5];

TEST_START = 0;
TEST_END =  5;

test_span = [TEST_START, TEST_END];

for i = 1:length(test_distrubance_rates)
    

    test_disturbance = wave(1/10,test_distrubance_rates(i));


    for j = 1:length(test_control_rates) %Do all the "static" controllers
        
        test_sampling_rate = test_control_rates(j);
        test_sampling_period = 1/test_sampling_rate;
        
        test_physical_system = inverted_pendulum(test_sampling_period,ps1_rates,ps1_initial_state);
        test_disturbance = wave(1/10,test_distrubance_rates(i));

        test_physical_system.add_disturbance(test_disturbance);
        
        test_traj = test_physical_system.simulate(test_span);

        ts = test_traj(:,end);
        xp3 = test_traj(:,3);

        traj_array{j,i} = test_traj;
        ase_array(j,i) = compute_ase(ts, xp3, zeros(length(ts)));
        compute_array(j,i) = test_sampling_rate * TEST_END;

    end

    %Do the co-regulated controller
    test_cps = multi_cps();
    cyber_system = simple_rotational(cs1_inertia,cs1_damping,cs1_control_input_range,cs1_lower_state_bounds,cs1_upper_state_bounds,cs1_sampling_period,cs1_initial_state);

    coreg_sys = basic_coreg(test_physical_system, cyber_system);
    test_cps.add_sub_system(coreg_sys)
    test_cps.add_disturbance(test_disturbance,1)

    test_traj = test_cps.simulate(test_span);

    ts = test_traj(:,end);
    xp3 = test_traj(:,test_cps.sub_systems{1}.cps_xpidcs(3)); %theta

    up1 = test_cps.sub_systems{1}.physical_system.input_updates(1,:);

    traj_array{j+1,i} = test_traj;
    ase_array(j+1,i) = compute_ase(ts, xp3, zeros(length(ts)));
    compute_array(j+1,i) = length(up1);
    

end

%% Analysis

ase_sums = [];
compute_sum = [];

for i = 1:length(ase_array)
    ase_sums(i) = sum(ase_array(i,:));
    compute_sum(i) = sum(compute_array(i,:));
end
figure;
hold on;
plot(compute_sum(2:end-1), ase_sums(2:end-1))
plot(compute_sum(end),ase_sums(end),'x');
hold off;
%% 

CONTROLLER_RATE = 21;
DISTURBANCE_RATE = 5;

trajectory = traj_array{CONTROLLER_RATE, DISTURBANCE_RATE};

ts = trajectory(:,end);
xp1 = trajectory(:,1);
xp2 = trajectory(:,2);
xp3 = trajectory(:,3); %angular position
xp4 = trajectory(:,4);
up = trajectory(:,7); %coreg
%up = trajectory(:,5); %static

figure;
hold on;
plot(ts,xp3);
plot(ts,up)
average_squared_error = compute_ase(ts, xp3, zeros(1,length(ts)));
title(average_squared_error)
legend(["theta","up"])
hold off;

%%
figure;
for i = 1:5
    trajectory = traj_array{21,i};
    
    ts = trajectory(:,end);
    xp1 = trajectory(:,1);
    xp2 = trajectory(:,2);
    xp3 = trajectory(:,3); %angular position
    xp4 = trajectory(:,4);
    xc1 = trajectory(:,5);
    xc2 = trajectory(:,6);
    up = trajectory(:,7); %coreg
    
    subplot(2,5,i)
    hold on;
    plot(ts,xp3);
    hold off;

    subplot(2,5,i+5)
    hold on;
    plot(ts,xc2);
    hold off;
    
end



%% Evaluation Functions
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


