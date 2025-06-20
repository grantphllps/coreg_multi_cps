classdef inverted_pendulum < lqr
    
    properties
        M; m; b; I; g; l; sys; gs; period_span;
    end

    methods
        function self = inverted_pendulum(sampling_period,rates,x0)
            self = self@lqr(sampling_period)
    
            M = .5;
            m = 0.2;
            b = 0.1;
            I = 0.006;
            g = 9.8;
            l = 0.3;

            self.x0 = x0;
    
            p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices
            self.A = [0      1              0           0;
                      0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
                      0      0              0           1;
                      0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
            
            self.B = [     0;
                 (I+m*l^2)/p;
                      0;
                    m*l/p];
            
            self.C = [1 0 0 0;
                 0 0 1 0];
            
            self.D = [0;
                 0];
    
            self.Q = eye(4);
            self.R = 1;

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
 
        end %inverted pendulum constructor

        function self = update_sampling_period(self, new_period)
            gs_idx = find(self.period_span <= new_period,1,"last");
            if isempty(gs_idx)
                [~, gs_idx] = min(self.period_span);
            end
            self.sampling_period = self.period_span(gs_idx);
            self.Kd = self.gs{gs_idx};

        end


    end

end
