classdef simple_rotational < mpc
    %Simple velocity-tracking rotational system

    properties 
        I; b; Q; R; velocity_reference;
    end %properties

    methods
        function self = simple_rotational(inertia, damping, input_constraints, lower_constraints, upper_constraints, sampling_period, x0, xref0)
            self = self@mpc(input_constraints, lower_constraints, upper_constraints, sampling_period);

            self.x0 = x0;

            if any(x0 > upper_constraints) || any(x0 < lower_constraints)
                error("rotational x0 outside of constraints")
            end

            self.I = inertia;
            self.b = damping;

            self.A = [0 1; 0 -self.b/self.I];
            self.B = [0; 1/self.I];
            self.C = eye(2);
            self.D = 0;
            self.Q = [0 0; 0 10]; %penalty for state
            self.R = 0;           %Penalty for input

            %self.velocity_reference = 10;
            self.velocity_reference = xref0;

        end % simple_rotational constructor

        function self = update_velocity_reference(self, new_reference)
            self.velocity_reference = new_reference;
        end % update_velocity_reference

        function J = cost(self,t,x,u)
            x_error = [0; self.velocity_reference] - x;
            J = x_error'*self.Q*x_error + u'*self.R*u;
        end %cost
        
        function self = update_sampling_period(self, new_period)
            self.sampling_period = new_period;
        end
    end %methods

end %classdef