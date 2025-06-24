classdef constant < disturbance
    properties

    end

    methods
        function self = constant(sampling_period)
            self = self@disturbance(sampling_period)
        end

        function output = compute_disturbance(self,t,x)
            if t > 5 && t < 15
                output = 1*sin(8 * pi * t);
            else
                output = 0;
            end
        end
    end
end