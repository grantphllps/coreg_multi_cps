classdef PeriodicTask < Task
    properties
        C
        T
        D
    end

    methods
        function obj = PeriodicTask(C, T, D, priority)
            if nargin < 3 || isempty(D)
                D = T;
            end
            if nargin < 4
                priority = [];
            end
            obj@Task(priority);
            obj.C = C;
            obj.T = T;
            obj.D = D;
        end

        function inter = interference(obj, t)
            if t <= 0
                inter = 0;
            else
                inter = ceil(t / obj.T) * obj.C;
            end
        end

        function sched = is_schedulable(obj, higher_tasks)
            r = obj.C;
            while true
                inter = 0;
                for ii = 1:length(higher_tasks)
                    inter = inter + higher_tasks(ii).interference(r);
                end
                new_r = obj.C + inter;
                if new_r == r
                    sched = (new_r <= obj.D);
                    return
                end
                if new_r > obj.D
                    sched = false;
                    return
                end
                r = new_r;
            end
        end
    end
end