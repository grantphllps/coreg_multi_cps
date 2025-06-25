classdef zero_disturbance < disturbance
    properties

    end

    methods
        function self = zero_disturbance(sampling_period)
            self = self@disturbance(inf)
        end

        function output = compute_disturbance(self,t,x)
            output = 0;
        end
    end %methods


end
