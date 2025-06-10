classdef (Abstract) lqr < control_system

    properties
        Q; R; Kd;
    end% properties

    methods
        function self = lqr(sampling_period)
            self = self@control_system(sampling_period)
        end % lqr constructor

        function xdot = systemfun(self,t,x,u)
            xdot = self.A*x + self.B*u;
        end %systemfun

        function u = systeminput(self,t,x)
            u = -self.Kd*x;
        end

    end %methods

end %classdef