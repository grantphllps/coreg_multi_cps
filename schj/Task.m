classdef Task < handle
    properties
        priority
    end

    methods
        function obj = Task(priority)
            obj.priority = priority;
        end

        function inter = interference(~, ~)
            error('Subclasses must implement interference(t)');
        end

        function sched = is_schedulable(~, ~)
            error('Subclasses must implement is_schedulable(higher_tasks)');
        end
    end
end