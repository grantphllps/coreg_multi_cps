classdef constant < disturbance
    properties
        offset = 0;
    end

    methods
        function self = constant(sampling_period)
            self = self@disturbance(sampling_period)
            self.offset = rand(1);
        end

        function output = compute_disturbance(self,t,x)
            
            
            if t > self.offset + 0 && t < self.offset + 10
                output = 3*sin(8 * pi * t);
            else
                output = 0;
            end
        end
    end
end