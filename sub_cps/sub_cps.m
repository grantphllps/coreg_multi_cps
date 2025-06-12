classdef (Abstract) sub_cps < handle

    properties
        physical_system; cyber_system; A; B; xp_idcs; xc_idcs; 
        cps_xpidcs; cps_xcidcs; % <- these are only used if part of a parent CPS
    end %properties

    methods
        function self = sub_cps(physical_system, cyber_system)
            self.physical_system = physical_system;
            self.cyber_system = cyber_system;

            self.A = blkdiag(physical_system.A, cyber_system.A);
            self.B = blkdiag(physical_system.B, cyber_system.B);
            
            %xps = # of states in physical system
            xps = length(physical_system.A(:,end));

            %xcs = # of states in cyber system
            xcs = length(cyber_system.A(:,end));

            self.xp_idcs = 1:xps;
            self.xc_idcs = (1:xcs) + length(self.xp_idcs);

            self.cps_xpidcs = -1;
            self.cps_xcidcs = -1;

        end

        function xdot = systemfun(self,t,x,u)
            xdot = self.A*x + self.B*u;
        end

        function trajectory = simulate(self, sim_span)
            
            sim_end = sim_span(2);
            window_start = sim_span(1);

            ref_switch = 1;

            % set the physical system sampling period
            self.physical_system.update_sampling_period( 1/ (self.cyber_system.x0(2)));
            
            % get the first update for the cyber-physical systems
            physical_system_update = window_start + self.physical_system.sampling_period;
            cyber_system_update = window_start + self.cyber_system.sampling_period;

            x_sim = [self.physical_system.x0; self.cyber_system.x0];
            t_sim = [0];
            u_sim = [0 0];

            u_p = 0;
            u_c = 0;
            
            update_switch = [1; 1];
            next_updates = [physical_system_update; cyber_system_update];
            
            while (sim_end > window_start)
                
                %1) set the window for this segement of the simulation
                [window_end, idx] = min(next_updates);
                window_span = window_start:0.001:window_end;

                %2) set the respective control inputs based on update
                %switch
                if update_switch(1) == 1
                    u_p = self.physical_system.systeminput(t_sim(end), x_sim(self.xp_idcs,end));
                end

                if update_switch(2) == 1
                    u_c = self.cyber_system.systeminput(t_sim(end), x_sim(self.xc_idcs,end));
                end
                update_switch = [0; 0]; %reset
                u = [u_p; u_c];

                %3) simulate the system through the window
                [t_window, x_window] = ode45( @(t_sim, x_sim) self.systemfun(t_sim,x_sim,u), window_span, x_sim(:,end));
                
                %4) update the *_sim vectors
                x_sim = [x_sim x_window'];
                t_sim = [t_sim t_window'];

                up_window = ones(length(t_window),1)*u_p;
                uc_window = ones(length(t_window),1)*u_c;
                u_window = [up_window uc_window];

                u_sim = [u_sim; u_window];


                %4) 
                self.physical_system.update_sampling_period( 1 / (x_sim(end,end)));
                self.cyber_system.update_velocity_reference(2*x_sim(1,end));

                for i = 1:length(next_updates)
                    if (next_updates(i) - t_sim(end) < 0.005)
                        if i == 1
                            next_updates(i) = next_updates(i) + self.physical_system.sampling_period;
                            update_switch(1) = 1;
                        elseif i == 2
                            next_updates(i) = next_updates(i) + self.cyber_system.sampling_period;
                            update_switch(2) = 1;
                        end
                    end
                end

                if t_sim(end) > 5 && ref_switch == 1
                    x_sim(1,end) = 7;
                    ref_switch = 0;
                end

                
                window_start = t_window(end);
            end %while
            trajectory = [x_sim', u_sim, t_sim'];
        end %simulate
    end %methods

end %classdef sub_sps 