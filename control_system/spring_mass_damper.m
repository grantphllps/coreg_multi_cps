classdef spring_mass_damper < lqr

    properties
        m; k; b; sys; gs; period_span;
    end

    methods
        function self = spring_mass_damper(mass,elasticity,damping,Q,R,sampling_period,rates,x0)
            self = self@lqr(sampling_period)

            self.m  = mass;
            self.b = damping;
            self.k = elasticity;
            self.x0 = x0;

            self.A = [0 1; -self.k/self.m -self.b/self.m];
            self.B = [0; 1/self.m];
            self.C = eye(length(self.A));
            self.D = 0;

            self.Q = Q;
            self.R = R;

            self.sys = ss(self.A,self.B,self.C,self.D);

            %Build the gain schedule
            self.period_span = flip(rates.^-1);
            for i = 1:length(self.period_span)
                dsys = c2d(self.sys, self.period_span(i),"zoh");
                self.gs{i} = dlqr(dsys.A,dsys.B,self.Q,self.R);
            end
            
            gs_idx = find(self.period_span <= sampling_period,1,"last");
            if isempty(gs_idx)
                [~, gs_idx] = min(self.period_span);
            end
             self.sampling_period = self.period_span(gs_idx);
            self.Kd = self.gs{gs_idx};
        
        end %spring_mass_damper constructor

        function self = update_sampling_period(self, new_period)
            gs_idx = find(self.period_span <= new_period,1,"last");
            if isempty(gs_idx)
                [~, gs_idx] = min(self.period_span);
            end
            self.sampling_period = self.period_span(gs_idx);
            self.Kd = self.gs{gs_idx};

        end % update_sampling_period
        
    end

end