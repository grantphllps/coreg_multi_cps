classdef (Abstract) mpc < control_system
    
    properties
        input_constraints; upper_constraints, lower_constraints; input_levels; horizon_length; sequences; controls;
    end %properties

    methods (Abstract) 
        J = cost(self,t,x,u)
    end %abstract methods

    methods
        function self = mpc(input_constraints, lower_constraints, upper_constraints,sampling_period)

            self = self@control_system(sampling_period)

            self.input_constraints = input_constraints;
            self.upper_constraints = upper_constraints;
            self.lower_constraints = lower_constraints;

            %just set to whatever, these seem to be resonable ...
            self.input_levels = 11;
            self.horizon_length = 2;

            self.controls = linspace(self.input_constraints(1),input_constraints(2), self.input_levels);
            self.sequences = generate_sequences(self.controls, self.horizon_length);
        end % mpc constructor

        function xdot = systemfun(self,t,x,u)
            xdot = self.A*x + self.B*u;
        end % systemfun

        function horizon_xdot = horizon_systemfun(self,t,x,u)
            xin = x(1:end-1); %remove the cost term
            xdot = self.systemfun(t,xin,u);
            J = self.cost(t,xin,u);
            horizon_xdot = [xdot; J]; %Add the updated cost
        end %horizon_systefun

        function u = systeminput(self,t,x)
            % MPC strategy, compute costs for all control sequences return
            % the first element of the sequence with the lowest cost
            % (invalid sequences will have an infinite cost)

            %1) Build the horizon steps
            horizon_start = t;
            horizon_end = horizon_start + self.horizon_length * self.sampling_period;
            horizon_steps = horizon_start:self.sampling_period:horizon_end;
            
            %2) Compute cost for all control sequences
            sequence_costs = inf*ones(length(self.sequences),1);

            for i = 1:length(self.sequences) %can also use "parfor" here
                
                %2.1) Compute the cost of "working sequence" over the
                %horizon
                working_sequence = self.sequences(i,:);
                x0_horizon = [x(:,end); 0]; %extra state for tracking cost
                
                % 2.1.1) Initalize system for this sequence/horizon
                x_horizon = x0_horizon;
                t_horizon = horizon_start;

                for j = 1:length(horizon_steps)-1
                    horizon_step_span = horizon_steps(j):0.002:horizon_steps(j+1);
                    u_horizon_step = working_sequence(j);

                    [t_horizon_step, x_horizon_step] = ode45( ...
                        @(t_horizon, x_horizon) self.horizon_systemfun(t_horizon,x_horizon,u_horizon_step), horizon_step_span,x_horizon(:,end));
                    
                    x_horizon = [x_horizon x_horizon_step'];
                    t_horizon = [t_horizon t_horizon_step'];

                end %simulate control over horizon step

                % Begin check for invalid sequences
                if any(any(x_horizon(1:end-1,:) > self.upper_constraints)) || any(any(x_horizon(1:end-1,:) < self.lower_constraints))
                    x_horizon(end, end) = inf;
                end
 
                sequence_costs(i) = x_horizon(end,end);

            end %sequences iterate
            
            %3) The sequence with the lowest cost is our control input
            [~ , min_idx] = min(sequence_costs);
            min_sequence = self.sequences(min_idx,:);
            u = min_sequence(1);
        end %systeminput
    end %methods

end %classdef

function sequences = generate_sequences(controls, horizon_length)
    N = numel(controls);
    grids = cell(1, horizon_length);
    [grids{:}] = ndgrid(1:N);
    indices = reshape(cat(horizon_length+1, grids{:}), [], horizon_length);  % N^horizon_length x horizon_length
    sequences = controls(indices);
end