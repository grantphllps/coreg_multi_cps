classdef (Abstract) control_system < handle

    properties
        A; B; C; D; x0; %Initial conditions
        sampling_period; %Time between physical input updates
        update_schedule; %time of next scheduled physical input update
        input_updates; %Vector tracking the time and value of physical control updates
        

        cps_state_idcs; % The indicies of this control system's state variables in the cps's state vector (x)
        cps_cntrl_idcs; % The indicies of this control system's control outputs in the cps's control vector (u)
        cps_update_idx; % The index of the state variable in the cps's state vector that represents this control system's sampling rate.
        cps_cntrl_idx;  % The inedx of this control system in the update_switch vector
    end

    methods
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
            
            sim_end = sim_span(2);

            window_start = sim_span(1);
            window_end = window_start + self.sampling_period;

            x_sim = self.x0;
            t_sim = [0];
            u_sim = [0]; 
            
            while (sim_end > window_start)

                window_span = window_start:0.001:window_end;
                u = self.systeminput(t_sim(end), x_sim(:,end));
                [t_window, x_window] = ode45( @(t_sim,x_sim) self.systemfun(t_sim,x_sim,u), window_span, x_sim(:,end));

                x_sim = [x_sim x_window'];
                t_sim = [t_sim t_window'];

                u_window = u*ones(length(t_window),1);
                u_sim = [u_sim; u_window];

                window_start = t_window(end);
                window_end = window_start + self.sampling_period;

            end

            trajectory = [x_sim', u_sim, t_sim'];

        end %simulate
    
    end %concrete methods

    methods (Abstract)
        xdot = systemfun(self,t,x,u)
        u = systeminput(self,t,x)
        %self = update_sampling_period(self,new_period)
    end %abstract methods

end % classdef control_system