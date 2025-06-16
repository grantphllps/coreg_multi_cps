classdef (Abstract) cps < handle

    properties
        sub_systems = {}; % Cell array of sub_cps that make up this system
        A; B;
        farts = {}; % holds handles to each control_system
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
            physical_state_idcs = state_idx_start:state_idx_start+new_physical_states-1;
            self.sub_systems{end}.cps_xpidcs = physical_state_idcs;

            self.farts{end+1} = new_sub_system.physical_system;
            self.farts{end}.cps_state_idcs = physical_state_idcs;
            self.sub_systems{end}.physical_system.cps_cntrl_idx = length(self.farts);
    
            % cyber states
            new_idx_start = state_idx_start+new_physical_states;
            cyber_state_idcs = new_idx_start:new_idx_start+new_cyber_states-1;
            self.sub_systems{end}.cps_xcidcs = cyber_state_idcs;
            
            %Assign the physical system update variable
            self.farts{end}.cps_update_idx = cyber_state_idcs(end);

            self.farts{end+1} = new_sub_system.cyber_system;
            self.farts{end}.cps_state_idcs = cyber_state_idcs;

            self.farts{end}.cps_update_idx = cyber_state_idcs(end);
            self.sub_systems{end}.cyber_system.cps_cntrl_idx = length(self.farts);
    
            
            % control indicies
            self.sub_systems{end}.cps_upidcs = cntrl_idx_start:cntrl_idx_start+new_physical_controls-1;
            new_idx_start = cntrl_idx_start + new_physical_controls;
            self.sub_systems{end}.cps_ucidcs = new_idx_start:new_idx_start+new_cyber_controls-1;


            % build the new A,B cps matrices
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

        function xdot = systemfun(self,t,x,u)
            xdot = self.A*x + self.B*u;
        end

        function [trajectory] = simulate(self, sim_span)
            
            sim_end = sim_span(2);
            window_start = sim_span(1);
            if (sim_end < window_start) || (sim_end - window_start) < 0.005 || length(sim_span) ~= 2
                error("Invalid simulation window")
            end
            
            % set the initial control system sampling rates
            for i = 1:length(self.sub_systems)
                rate = self.sub_systems{i}.cyber_system.x0(2); % Assumption: the second state var of the cyber system represents the update rate Hz
                starting_period = 1/rate;
                self.sub_systems{i}.physical_system.update_sampling_period(starting_period);
            end
            
            % set the initial "next update" vector
            number_of_control_systems = length(self.farts);
            next_update = zeros(number_of_control_systems,1);
            
            % Start Old Version
            % for i = 1:length(self.sub_systems)
            %     idx = 2*i - 1; %Assumption: exactly 1 cyber and 1 physical control per subsystem 
            % 
            %     physical_system_update = window_start + self.sub_systems{i}.physical_system.sampling_period;
            %     cyber_system_update = window_start + self.sub_systems{i}.cyber_system.sampling_period;
            % 
            %     next_update(idx) = physical_system_update;
            %     next_update(idx + 1) = cyber_system_update;
            % end
            % End Old Version

            % Start New Version
            for i = 1:length(self.farts)
                next_control_update = window_start + self.farts{i}.sampling_period;
                self.farts{i}.update_sampling_period(next_control_update);
                self.farts{i}.set_next_update(next_control_update)
                next_update(i) = next_control_update;
            end
            % End New Version

            % set the intial state, lots of dimensional assumptions here
           
            % Old version
            % [number_of_states, ~] = size(self.A);
            % x_sim = zeros(number_of_states,1);
            % 
            % 
            % for i = 1:length(self.sub_systems)
            %     idcs = self.sub_systems{i}.cps_xpidcs;
            %     x_sim(idcs) = self.sub_systems{i}.physical_system.x0;
            %     idcs = self.sub_systems{i}.cps_xcidcs;
            %     x_sim(idcs) = self.sub_systems{i}.cyber_system.x0;
            % end 

            x_sim = [];

            %New Version
            for i = 1:length(self.farts)
                new_x0 = self.farts{i}.x0;
                x_sim = [x_sim; new_x0];
            end
            

            % set other simulation variables
            t_sim = sim_span(1);
            [~, number_of_inputs] = size(self.B);
            u_sim = zeros(1,number_of_inputs);
            u = zeros(number_of_inputs,1);
            update_switch = ones(1,number_of_inputs);

            % TODO: get rid of these
            up1_updates = [];
            up1_time = [];
            up2_updates = [];
            up2_time = [];
            while (sim_end > window_start)
            
                %1) set the window for this segment of the simulation
                [window_end, ~] = min(next_update);
                window_span = window_start:0.001:window_end;

                %2) set the respective control inputs based on the update
                %switch, very hard-coded
                % if update_switch(1) == 1
                %     idcs = self.sub_systems{1}.cps_xpidcs;
                %     working_x = x_sim(idcs,end);
                %     u(1) = self.sub_systems{1}.physical_system.systeminput(t_sim(end), working_x);
                % 
                %     up1_time(end+1) = t_sim(end);
                %     up1_updates(end+1) = u(1);
                % 
                % end
                % 
                % if update_switch(2) == 1
                %     idcs = self.sub_systems{1}.cps_xcidcs;
                %     working_x = x_sim(idcs,end);
                %     u(2) = self.sub_systems{1}.cyber_system.systeminput(t_sim(end), working_x);
                % end
                % 
                % if update_switch(3) == 1
                %     idcs = self.sub_systems{2}.cps_xpidcs;
                %     working_x = x_sim(idcs,end);
                %     u(3) = self.sub_systems{2}.physical_system.systeminput(t_sim(end), working_x);
                % 
                %     up2_time(end+1) = t_sim(end);
                %     up2_updates(end+1) = u(3);
                % end
                % 
                % if update_switch(4) == 1
                %     idcs = self.sub_systems{2}.cps_xcidcs;
                %     working_x = x_sim(idcs,end);
                %     u(4) = self.sub_systems{2}.cyber_system.systeminput(t_sim(end), working_x);
                % end

                % Set the respective control inputs based on the update switch
                for i = 1:length(self.farts) 
                    if update_switch(i) == 1
                        %calculate the new control input
                        idcs = self.farts{i}.cps_state_idcs;
                        working_x = x_sim(idcs,end);
                        u(i) = self.farts{i}.systeminput(t_sim(end), working_x);
                        %track the new update and time it occured
                        self.farts{i}.input_updates(:,end+1) = zeros(length(u(i))+1,1);
                        self.farts{i}.input_updates(1,end) = t_sim(end);
                        self.farts{i}.input_updates(2:end,end) = u(i);

                    end
                end
                
                update_switch = 0*update_switch; %reset update switch

                 %3) simulate the system through the window
                [t_window, x_window] = ode45( @(t_sim, x_sim) self.systemfun(t_sim,x_sim,u), window_span, x_sim(:,end));
                
                %4) update the *_sim vectors
                x_sim = [x_sim x_window'];
                t_sim = [t_sim t_window'];

                u_window = -1*ones(length(t_window), number_of_inputs);

                for i = 1:number_of_inputs
                    u_window(:,i) = u(i)*u_window(:,i);
                end

                u_sim = [u_sim; u_window];

                %4) update stuff
                % for i = 1:length(next_update)
                %     if (next_update(i) - t_sim(end) < 0.005)
                %         if i == 1
                %             % update ps1 update rate
                %             rate_idx = self.sub_systems{1}.cps_xcidcs(2);
                %             new_rate = x_sim(rate_idx,end);
                %             new_period = 1 / new_rate;
                %             self.sub_systems{1}.physical_system.update_sampling_period(new_period);
                % 
                %             % update ps control
                %             update_switch(i) = 1;
                % 
                %             % update next_update
                %             next_update(i) = next_update(i) + self.sub_systems{1}.physical_system.sampling_period;
                % 
                %         elseif i == 2
                %             % update cs1 target
                %             state_idx = self.sub_systems{1}.cps_xpidcs(1);
                %             new_rate_target = 2 * x_sim(state_idx,end); % arbitrary coupling
                %             self.sub_systems{1}.cyber_system.update_velocity_reference(new_rate_target);
                % 
                %             % update cs control
                %             update_switch(i) = 1;
                % 
                %             % update next_update
                %             next_update(i) = next_update(i) + self.sub_systems{1}.cyber_system.sampling_period;
                % 
                %         elseif i == 3
                %             % update ps2 update rate
                %             rate_idx = self.sub_systems{2}.cps_xcidcs(2);
                %             new_rate = x_sim(rate_idx,end);
                %             new_period = 1 / new_rate;
                %             self.sub_systems{2}.physical_system.update_sampling_period(new_period);
                % 
                %             % update ps control
                %             update_switch(i) = 1;
                % 
                %             % update next_update
                %             next_update(i) = next_update(i) + self.sub_systems{2}.physical_system.sampling_period;
                % 
                %         elseif i == 4
                %             % update cs1 target
                %             state_idx = self.sub_systems{2}.cps_xpidcs(1);
                %             new_rate_target = 2 * abs(x_sim(state_idx,end)); % arbitrary coupling
                %             self.sub_systems{2}.cyber_system.update_velocity_reference(new_rate_target);
                % 
                %             % update cs control
                %             update_switch(i) = 1;
                % 
                %             % update next_update
                %             next_update(i) = next_update(i) + self.sub_systems{2}.cyber_system.sampling_period;
                %         end
                %     end %time check
                % end %update loop

                %New update loop
                for i = 1:length(self.sub_systems)

                    if ( (self.sub_systems{i}.physical_system.next_update - t_sim(end) ) <= 0.005)
                        rate_idx = self.sub_systems{i}.physical_system.cps_update_idx;
                        new_rate = x_sim(rate_idx, end);
                        new_period = 1 / new_rate;
                        self.sub_systems{i}.physical_system.update_sampling_period(new_period);

                        switch_idx = self.sub_systems{i}.physical_system.cps_cntrl_idx;
                        update_switch(switch_idx) = 1;
                        
                        next_update = self.sub_systems{i}.physical_system.next_update + self.sub_systems{i}.physical_system.sampling_period;
                        self.sub_systems{i}.physical_system.set_next_update(next_update)
                    end

                    if ( (self.sub_systems{i}.cyber_system.next_update - t_sim(end) ) <= 0.005)
                        state_idx = self.sub_systems{i}.cps_xpidcs(1);
                        new_rate_target = 2* x_sim(state_idx,end);
                        self.sub_systems{i}.cyber_system.update_velocity_reference(new_rate_target);

                        switch_idx = self.sub_systems{i}.cyber_system.cps_cntrl_idx;
                        update_switch(switch_idx) = 1;

                        next_update = self.sub_systems{i}.cyber_system.next_update + self.sub_systems{i}.cyber_system.sampling_period;
                        self.sub_systems{i}.cyber_system.set_next_update(next_update)
                    end

                end % New update loop

                for i = 1:length(self.farts)
                    next_update(i) = self.farts{i}.next_update;
                end

                window_start = t_window(end)
            end %while
        trajectory = [x_sim', u_sim, t_sim'];
        end %simulate
    end % methods
end % CPS classdef