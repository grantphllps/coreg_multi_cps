function [pri_order, schedulable] = opa(taskset, sched_test)
% [pri_order, schedulable] = opa(taskset, sched_test)
%   Implements Audsley's OPA algorithm.
%   - taskset:    a struct array of tasks
%   - sched_test: a function handle of the form
%                   ok = sched_test(taskset, order)
%                 where 'order' is an index array (highest to lowest priority)
%
% Returns:
%   - pri_order:      array of task indices, from highest to lowest priority
%   - schedulable:    true/false

n = numel(taskset);
remaining = 1:n;
pri_order = zeros(1,n);
schedulable = true;

for level = n:-1:1    % Assign the lowest available priority each time
    placed = false;
    for idx = remaining
        trial_order = [setdiff(remaining, idx, 'stable'), idx]; % idx as lowest
        if sched_test(taskset, trial_order)
            pri_order(level) = idx;             % assign 'idx' to current lowest
            remaining(remaining == idx) = [];   % remove it from unassigned
            placed = true;
            break
        end
    end
    if ~placed
        % No task could be assigned this priority: not schedulable
        schedulable = false;
        pri_order = [];
        return
    end
end

% Reorder pri_order to be highest priority first (Audsley's convention)
pri_order = fliplr(pri_order);
end
