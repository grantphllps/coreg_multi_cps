function schedulable = rta_sp_schedulability(taskset)
    % Performs the RTa-sp schedulability test on a VRB task set.
    % taskset is a cell array of structs, each with 'modes' as Mx3 matrix [C, D, T].
    % Returns true if the task set is schedulable under fixed priority scheduling
    % using the RTa-sp test, which treats each VRB task as a sporadic task with
    % C = max(C_m), T = min(T_m), D = min(D_m).
    % Priorities assigned via deadline monotonic (smaller min D higher priority).
    % Assumes no blocking (B_i = 0).

    n = length(taskset);
    if n == 0
        schedulable = true;
        return;
    end
    
    % Compute min_D for each task (since D_m = T_m, min_D = min_T)
    min_Ds = zeros(1, n);
    for i = 1:n
        min_Ds(i) = min(taskset{i}.modes(:, 2));  % min D_m
    end
    
    % Sort task indices by increasing min_D (highest priority first)
    [~, priority_order] = sort(min_Ds);  % priority_order(1) has smallest min_D
    
    % For each task in priority order
    for k = 1:n
        tau_k_idx = priority_order(k);
        tau_k = taskset{tau_k_idx};
        
        % Sporadic parameters for task k
        C_k = max(tau_k.modes(:, 1));
        D_k = min(tau_k.modes(:, 2));
        
        % Higher priority tasks indices
        hp_indices = priority_order(1:k-1);
        
        % Sporadic parameters for hp tasks
        hp_C = zeros(1, k-1);
        hp_T = zeros(1, k-1);
        for j = 1:k-1
            tau_j_idx = hp_indices(j);
            tau_j = taskset{tau_j_idx};
            hp_C(j) = max(tau_j.modes(:, 1));
            hp_T(j) = min(tau_j.modes(:, 3));  % T_j = min T_m
        end
        
        % Response time iteration
        w = C_k;
        prev_w = 0;
        while w ~= prev_w
            interf = sum(ceil(w ./ hp_T) .* hp_C);
            new_w = C_k + interf;
            if new_w > D_k
                schedulable = false;
                return;
            end
            prev_w = w;
            w = new_w;
        end
        R_k = w;
        if R_k > D_k
            schedulable = false;
            return;
        end
    end
    schedulable = true;
end