classdef (Abstract) control_system < handle

    properties
        A; B; C; D; sampling_period; x0;
    end

    methods
        function self = control_system(sampling_period)
            self.sampling_period = sampling_period;
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
    end %abstract methods

end % classdef control_system