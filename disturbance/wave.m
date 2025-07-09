classdef wave < disturbance
    properties
        offset = 0;
        frequencey = 0;
    end

    methods
        function self = wave(sampling_period, frequencey)
            self = self@disturbance(sampling_period)
            self.offset = rand(1);
            self.frequencey = frequencey;
        end

        function output = compute_disturbance(self,t,x)
           
            if t > self.offset + 0 && t < self.offset
                output = 2*abs(sin(1 * self.frequencey * pi * t));
            else
                output = 0;
            end
        end
    end
end