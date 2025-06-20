classdef (Abstract) cps < handle

    properties
        sub_systems = {}; % Cell array of sub_cps that make up this system
        A; B;
        control_systems = {}; % holds handles to each control_system
        disturbances = {};
    end

    methods
        function self = cps()

        end % %init_parent_system

        function self = add_sub_system(self, new_sub_system)
            state_idx_start = length(self.A) + 1;
            cntrl_idx_start = length(self.B) + 1;

            self.sub_systems{end+1} = new_sub_system;
            
            % Assign state and control idxs
            [new_physical_states, ~] = size(new_sub_system.physical_system.A);
            [new_cyber_states, ~] = size(new_sub_system.cyber_system.A);
            [~, new_physical_controls] = size(new_sub_system.physical_system.B);
            [~, new_cyber_controls] = size(new_sub_system.cyber_system.B);


            % state indicies
            
            % physical states
            physical_state_idcs = state_idx_start:state_idx_start+new_physical_states-1; % indicies of this physical system's state in cps state vector
            self.sub_systems{end}.cps_xpidcs = physical_state_idcs;

            self.control_systems{end+1} = new_sub_system.physical_system; % Add this control system to 'self.control_systems'
            self.control_systems{end}.cps_state_idcs = physical_state_idcs; % indicies of this control system's state in cps state vector (as property of the control system) (redundant?)
            self.sub_systems{end}.physical_system.cps_cntrl_idx = length(self.control_systems); % indicies of this control system in 'self.control_systems'
    
            % cyber states
            new_idx_start = state_idx_start+new_physical_states;
            cyber_state_idcs = new_idx_start:new_idx_start+new_cyber_states-1; % indicies of this cyber system's state in cps state vector
            self.sub_systems{end}.cps_xcidcs = cyber_state_idcs; 
            
            %Assign the physical system update variable
            self.control_systems{end}.cps_update_idx = cyber_state_idcs(end); % the index of the cyber state variable that sets the physical system's update rate

            %helpful vector of handles to sub_system control systems
            self.control_systems{end+1} = new_sub_system.cyber_system;
            self.control_systems{end}.cps_state_idcs = cyber_state_idcs;

            self.control_systems{end}.cps_update_idx = cyber_state_idcs(end); %indicies of the state variable used as the update rate for this control system (should not affect a cyber system)
            self.sub_systems{end}.cyber_system.cps_cntrl_idx = length(self.control_systems); % indicies of this control system in 'self.control_systems'
    
            
            % control indicies
            self.sub_systems{end}.cps_upidcs = cntrl_idx_start:cntrl_idx_start+new_physical_controls-1; %indicies of sub system's physical control inputs
            new_idx_start = cntrl_idx_start + new_physical_controls;
            self.sub_systems{end}.cps_ucidcs = new_idx_start:new_idx_start+new_cyber_controls-1; %indicies of sub system's cyber control inputs


            % build the new A,B matrices for the entire cps
            newA = [];
            newB = [];
            for i = 1:length(self.sub_systems)
                newA = blkdiag(newA, self.sub_systems{i}.physical_system.A);
                newA = blkdiag(newA, self.sub_systems{i}.cyber_system.A);

                newB = blkdiag(newB, self.sub_systems{i}.physical_system.B);
                newB = blkdiag(newB, self.sub_systems{i}.cyber_system.B);
            end
            
            self.A = newA;
            self.B = newB;
        end

        function self = add_disturbance(self, new_disturbance, binding)
            self.disturbances{end+1} = new_disturbance;
            
            % NOT working
            new_disturbance.row = binding;
            new_disturbance.col = length(self.disturbances);
            
        end

        function xdot = systemfun(self,t,x,u)
            xdot = self.A*x + self.B*u;
        end

        function [trajectory] = simulate(self, sim_span)
            % The CPS is simulated by breaking the entire 'sim_span' into
            % smaller windows that end when the next control system within
            % the CPS needs to update its control input (cyber or physical)
            
            %1) set the simulation window, check that it's valid 
            sim_end = sim_span(2);
            window_start = sim_span(1);
            if (sim_end < window_start) || (sim_end - window_start) < 0.005 || length(sim_span) ~= 2
                error("Invalid simulation window")
            end
            
            %2) set the initial 'sampling_period' for the physical systems
            % based on the designated cyber state variable
            for i = 1:length(self.sub_systems)
                rate = self.sub_systems{i}.cyber_system.x0(2); % Assumption: the second state var of the cyber system represents the update rate Hz
                starting_period = 1/rate;
                self.sub_systems{i}.physical_system.update_sampling_period(starting_period);
            end
            
            %3) initialize the 'update_schedule' vector that holds the update
            %times for each control system
            number_of_control_systems = length(self.control_systems);
            control_update_schedule = zeros(number_of_control_systems,1);
            for i = 1:length(self.control_systems)
                self.control_systems{i}.refresh_update_schedule(window_start);
                control_update_schedule(i) = self.control_systems{i}.update_schedule;
            end

            %4) Initialize CPS initial conditions
            x_sim = [];
            for i = 1:length(self.control_systems)
                new_x0 = self.control_systems{i}.x0;
                x_sim = [x_sim; new_x0];
            end
            

            %5) set other simulation variables
            t_sim = sim_span(1); % initial simulation time
            [~, number_of_controls] = size(self.B);
            u_sim = zeros(1,number_of_controls); %control input history
            u_controls = zeros(number_of_controls,1); %control input vector (not the history)
            control_update_switch = ones(1,number_of_controls); %logical vector that indicates if a control system should be updated
            
            %6) Disturbance stuff
            disturbance_rows = number_of_controls;
            disturbance_cols = length(self.disturbances);
            
            if disturbance_cols == 0
                disturbance_cols = 1;
            end

            %Assumptions happinging here
            disturbance_mapping = zeros(disturbance_rows);
            disturbance_update_schedule = [];
            disturbance_vals = zeros(disturbance_rows, 1);

            for i = 1:length(self.disturbances)
                self.disturbances{i}.refresh_update_schedule(window_start)
                disturbance_update_schedule(i) = self.disturbances{i}.update_schedule;
                disturbance_row = self.disturbances{i}.row;
                disturbance_col = self.disturbances{i}.col;
                disturbance_mapping(disturbance_row,disturbance_col) = 1;
            end
            disturbance_update_switch = ones(1,disturbance_cols);

            %6) simulate CPS evolution through the windows
            while (sim_end > window_start)
            
                %6.1) set the window for this segment of the simulation
                update_schedule = [control_update_schedule; disturbance_update_schedule];
                [window_end, ~] = min(update_schedule);
                window_span = window_start:0.001:window_end;

                %6.2) set the control inputs based on the update switch
                for i = 1:length(self.control_systems) 
                    if control_update_switch(i) == 1
                        %calculate the new control input
                        idcs = self.control_systems{i}.cps_state_idcs; %indicies of this control system's state(s)
                        working_x = x_sim(idcs,end);
                        u_controls(i) = self.control_systems{i}.systeminput(t_sim(end), working_x);
                        %track the new update and time it occured
                        self.control_systems{i}.input_updates(:,end+1) = zeros(length(u_controls(i))+1,1);
                        self.control_systems{i}.input_updates(1,end) = t_sim(end);
                        self.control_systems{i}.input_updates(2:end,end) = u_controls(i);

                    end
                end

                for i = 1:length(self.disturbances)
                    if disturbance_update_switch(i) == 1
                        disturbance_vals(i) = self.disturbances{i}.compute_disturbance(t_sim(end));
                    end
                end

                u = u_controls+disturbance_mapping * disturbance_vals;
                
                %6.3) reset update switch
                control_update_switch = 0*control_update_switch; 
                disturbance_update_swtich = 0*disturbance_update_switch;

                %6.4) simulate the system through the window
                [t_window, x_window] = ode45( @(t_sim, x_sim) self.systemfun(t_sim,x_sim,u), window_span, x_sim(:,end));
                
                %6.5) update the history vectors
                x_sim = [x_sim x_window'];
                t_sim = [t_sim t_window'];
                
                %6.6) the control inputs are zoh's, build their 'history'
                u_window = 1*ones(length(t_window), number_of_controls);
                for i = 1:number_of_controls
                    u_window(:,i) = u_controls(i)*u_window(:,i);
                end
                u_sim = [u_sim; u_window];

                %6.7) Update sampling rates, schedule next updates, set update
                % switch as needed (with a 0.005 sec buffer)
                for i = 1:length(self.sub_systems)

                    if ( (self.sub_systems{i}.physical_system.update_schedule - t_sim(end) ) <= 0.005)
                        rate_idx = self.sub_systems{i}.physical_system.cps_update_idx;
                        new_rate = x_sim(rate_idx, end);
                        new_period = 1 / new_rate;
                        self.sub_systems{i}.physical_system.update_sampling_period(new_period);

                        switch_idx = self.sub_systems{i}.physical_system.cps_cntrl_idx;
                        control_update_switch(switch_idx) = 1;
                        
                        self.sub_systems{i}.physical_system.refresh_update_schedule(t_sim(end))
                    end

                    if ( (self.sub_systems{i}.cyber_system.update_schedule - t_sim(end) ) <= 0.005)
                        
                        % CPS coupling
                        physical_system_state = x_sim(self.sub_systems{i}.cps_xpidcs,end);
                        new_rate_target = norm(physical_system_state);
                        self.sub_systems{i}.cyber_system.update_velocity_reference(new_rate_target);
                        % CPS coupling end

                        switch_idx = self.sub_systems{i}.cyber_system.cps_cntrl_idx;
                        control_update_switch(switch_idx) = 1;

                        self.sub_systems{i}.cyber_system.refresh_update_schedule(t_sim(end))
                    end

                end

                for i = 1:length(self.disturbances)
                    if ((self.disturbances{i}.update_schedule - t_sim(end) ) <= 0.005)
                        self.disturbances{i}.refresh_update_schedule(t_sim(end))
                        disturbance_update_switch(i) = 1;
                    end
                end
                
                %6.8) refresh 'update_schedule' vector
                for i = 1:length(self.control_systems)
                    control_update_schedule(i) = self.control_systems{i}.update_schedule;
                end

                for i = 1:length(self.disturbances)
                    disturbance_update_schedule(i) = self.disturbances{i}.update_schedule;
                end
                
                %6.9) set the nexxt window start
                window_start = t_window(end)
            end %6)

        trajectory = [x_sim', u_sim, t_sim'];
        end %simulate
    end % methods
end % CPS classdef