classdef (Abstract) disturbance <handle

    properties
        sampling_period;
        update_schedule;
        row;
        col;
    end
    
    methods
        function self = disturbance(sampling_period)
            self.sampling_period = sampling_period;
            self.update_schedule = sampling_period;
        end

        function self = refresh_update_schedule(self, t_now)
            if (t_now - self.update_schedule) < 0.005
                self.update_schedule = t_now + self.sampling_period;
            end
        end
    end

    methods (Abstract)
        output = compute_disturbance(self,t,x)
    end

end