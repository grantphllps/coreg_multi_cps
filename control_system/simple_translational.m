classdef simple_translational < lqr

    properties
        M; b; sys; gs; period_span;
    end

    methods
        function self = simple_translational(mass,damping,Q,R,sampling_period,x0)
            self = self@lqr(sampling_period)

            self.M = mass;
            self.b = damping;
            self.x0 = x0;

            self.A = [0 1; 0 -self.b/self.M];
            self.B = [0; 1/self.M];
            self.C = eye(2);
            self.D = 0;

            self.Q = Q;
            self.R = R;

            self.sys = ss(self.A,self.B,self.C,self.D);
            
            %un-hardcode this someday
            self.period_span = (20:-1:2).^-1;
            for i = 1:length(self.period_span)
                dsys = c2d(self.sys, self.period_span(i),"zoh");
                self.gs{i} = dlqr(dsys.A,dsys.B,self.Q,self.R);
            end

            self.Kd = self.gs{1};
 
        end %simple_translational constructor

        function self = update_sampling_period(self, new_period)
            gs_idx = find(self.period_span <= new_period,1,"last");
            if isempty(gs_idx)
                [~, gs_idx] = min(self.period_span);
            end
            self.sampling_period = self.period_span(gs_idx);
            self.Kd = self.gs{gs_idx};
            disp(self.sampling_period)

        end % update_sampling_period

        
    end
 
end % classdef