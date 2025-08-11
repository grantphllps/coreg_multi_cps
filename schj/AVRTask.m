classdef AVRTask < Task
    properties
        calculator
        delta_theta
    end

    methods
        function obj = AVRTask(modes, max_period_ms, alpha_minus, alpha_plus, delta_theta, priority)
            if nargin < 5 || isempty(delta_theta)
                delta_theta = 60000.0;
            end
            if nargin < 6
                priority = [];
            end
            obj@Task(priority);
            obj.calculator = AVRInterferenceCalculator(modes, max_period_ms, alpha_minus, alpha_plus, delta_theta);
            obj.delta_theta = delta_theta;
        end

        function inter = interference(obj, t)
            inter = obj.calculator.worst_case_interference(t);
        end

        function sched = is_schedulable(obj, higher_tasks, D_func)
            if nargin < 3
                D_func = @(T) T;
            end
            for ii = 1:length(obj.calculator.boundaries)
                omega = obj.calculator.boundaries(ii);
                C = obj.calculator.get_C(omega);
                T = obj.delta_theta / omega;
                D = D_func(T);
                r = C;
                while true
                    inter = 0;
                    for jj = 1:length(higher_tasks)
                        inter = inter + higher_tasks(jj).interference(r);
                    end
                    new_r = C + inter;
                    if new_r == r
                        if new_r > D
                            sched = false;
                            return
                        end
                        break
                    end
                    if new_r > D
                        sched = false;
                        return
                    end
                    r = new_r;
                end
            end
            sched = true;
        end
    end
end

