classdef constant < disturbance
    properties

    end

    methods
        function self = constant(sampling_period)
            self = self@disturbance(sampling_period)
        end

        function output = compute_disturbance(self,t,x)
            output = 8*sin(8 * pi * t);
        end
    end
end