classdef (Abstract) control_system < handle

    properties
        A; B; C; D; x0; %Initial conditions
        sampling_period; %Time between physical input updates
        update_schedule; %time of next scheduled physical input update
        input_updates; %Vector tracking the time and value of physical control updates
        
        
        disturbance = {};

        ref_trajectory = [];
        ref_update_schedule; %next time to update the reference
        ref_idx = 1;
        ref = [];

        cps_state_idcs; % The indicies of this control system's state variables in the cps's state vector (x)
        cps_cntrl_idcs; % The indicies of this control system's control outputs in the cps's control vector (u)
        cps_update_idx; % The index of the state variable in the cps's state vector that represents this control system's sampling rate.
        cps_cntrl_idx;  % The inedx of this control system in the update_switch vector
    end

    methods
        function self = add_reference(self, new_reference)
            ref_to_add = new_reference;
            [rows, cols] = size(new_reference);
            ref_to_add(end+1,:) = inf*ones(1,cols);

            self.ref_trajectory = ref_to_add;
            self.ref_update_schedule = new_reference(1,1);
        end

        function self = update_reference(self)
            new_traj = self.ref_trajectory(self.ref_idx,:);
            new_ref = new_traj(2:end);
            self.ref = new_ref;
        end

        function self = update_ref_schedule(self)
            self.ref_idx = self.ref_idx + 1;
            next_time = self.ref_trajectory(self.ref_idx,1);
            self.ref_update_schedule = next_time;
        end

        function self = add_disturbance(self,new_disturbance)
            self.disturbance = new_disturbance;
        end

        function self = control_system(sampling_period)
            self.sampling_period = sampling_period;
            self.update_schedule = 0;
            self.input_updates = [];
            self.cps_state_idcs = -1;
            self.cps_cntrl_idcs = -1;
            self.cps_update_idx = -1;
            self.cps_cntrl_idx = -1;
            
        end

        function self = refresh_update_schedule(self, t_now)
            if (t_now - self.update_schedule) < 0.005
                self.update_schedule = t_now + self.sampling_period;
            end
        end

        function self = update_sampling_period(self, new_period)
            self.sampling_period = new_period;
        end
        
        function schedule = get_update_schedule(self)
            schedule = self.update_schedule;
        end

        function trajectory = simulate(self, sim_span)
            
            % set the simulation window
            sim_start = sim_span(1);
            sim_end = sim_span(2);
            window_start = sim_span(1);

            %Disturbance
            if isempty(self.disturbance)
                self.disturbance = zero_disturbance(500);
            end

            self.refresh_update_schedule(window_start);
            self.disturbance.refresh_update_schedule(window_start);

            
            control_update = self.update_schedule;
            disturbance_update = self.disturbance.update_schedule;
            
            update_schedule = [control_update; disturbance_update]; %note this is not self.update_schedule
            update_switch = [1 1];
            
            x_sim = self.x0;
            t_sim = [0];
            u_sim = [0]; 
            
            while (sim_end > window_start)

                window_end = min(update_schedule);
                window_span = window_start:0.001:window_end;

                if update_switch(1) == 1

                    if (window_start - self.ref_update_schedule) > 0
                        % update the reference
                        self.update_reference();
                        
                        % update the reference schedule
                        self.update_ref_schedule();

                        x_sim(:,end) = self.ref;

                    end

                    u_control = self.systeminput(t_sim(end), x_sim(:,end));
                    
                end

                if update_switch(2) == 1
                    u_dist = self.disturbance.compute_disturbance(t_sim(end), x_sim(:,end));
                end

                update_switch = [0, 0];
                u = u_control + u_dist;
                [t_window, x_window] = ode45( @(t_sim,x_sim) self.systemfun(t_sim,x_sim,u), window_span, x_sim(:,end));

                x_sim = [x_sim x_window'];
                t_sim = [t_sim t_window'];

                u_window = u_control*ones(length(t_window),1);
                u_sim = [u_sim; u_window];

                t_now = t_sim(end);

                if self.update_schedule - t_sim(end) < 0.005
                    self.refresh_update_schedule(t_now);
                    update_switch(1) = 1;
                end

                if self.disturbance.update_schedule - t_sim(end) < 0.005
                    self.disturbance.refresh_update_schedule(t_now);
                    update_switch(2) = 1;
                end

                window_start = t_now;
                update_schedule = [self.update_schedule; self.disturbance.update_schedule];

            end

            trajectory = [x_sim', u_sim, t_sim'];

        end %simulate
    
    end %concrete methods

    methods (Abstract)
        xdot = systemfun(self,t,x,u)
        u = systeminput(self,t,x)
    end %abstract methods

end % classdef control_system