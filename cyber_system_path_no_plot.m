close all;
clear;
clc;

%CPS1 components

inertia = 6.9137;
damping = 50;
control_input_range = [-500, 2000];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 10];
sampling_period = 1/10;
initial_state = [0; 2.01];

cyber_system1 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state,2.01);

mass = 1;
damping = 1;
Q = eye(2);
R = 1;
sampling_period = 2.01;
initial_state = [5; 0];
rates = 1:1:20;

physical_system1 = simple_translational(mass,damping,Q,R,sampling_period,rates,initial_state);

%Inverted pendulum is x dx theta dtheta

ps1_initial_state = [1; 0; 0.3; 0];
physical_system11 = inverted_pendulum(sampling_period,rates,ps1_initial_state);

%CPS2 components

inertia = 6.9137;
damping = 50;
control_input_range = [-500, 2000];
lower_state_bounds = [-inf; 1];
upper_state_bounds = [inf; 10];
sampling_period = 1/10;
initial_state = [0; 8];

cyber_system2 = simple_rotational(inertia, damping, control_input_range, lower_state_bounds, upper_state_bounds, sampling_period, initial_state,8);

m = 1;
k = 632;
b = 1;
Q = eye(2);
R = 1;
sampling_period = 8;
initial_state = [0; 0];
rates = 1:1:20;

ps2_initial_state = [-.3; 0; -0.3; 0];
physical_system2 = inverted_pendulum(sampling_period,rates,ps2_initial_state);

sub_2 = basic_coreg(physical_system2, cyber_system2);
sub_1 = basic_coreg(physical_system11, cyber_system1);

cps = multi_cps();
cps.add_sub_system(sub_1);
cps.add_sub_system(sub_2);

% cps.set_cyber_system_trajectory({[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[1.98,8],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[8,1.98],[4,4]})
cps.set_cyber_system_trajectory({[1.98,8],[1.98,8],[1.98,8],[8,1.98]})

%% Run the simulation

[trajectory] = cps.simulate([0,2]);

% Define the folder where you want to save
folder = './';

% Make sure the folder exists (create if not)
if ~exist(folder, 'dir')
    mkdir(folder);
end

% Define the filename
filename = fullfile(folder, 'myWorkspace.mat');

% Save the entire workspace
save(filename);
