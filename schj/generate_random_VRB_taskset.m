function taskset = generate_random_VRB_taskset(n, num_modes, total_U)
    % Generates a random VRB task set with n tasks, num_modes modes per task, and total utilization total_U.
    % Uses parameters from the paper: periods log-uniform in [10, 1000] ms, scale_f = 2, adjust_e = 0.2.
    % Implicit deadlines (D_m = T_m). No sporadic or baseline tasks.
    % taskset is a cell array of structs, each with 'modes' as Mx3 matrix [C, D, T].

    % Fixed parameters from the paper
    min_T_log10 = 1;    % log10(10 ms)
    max_T_log10 = 3;    % log10(1000 ms)
    scale_f = 2;        % Scaling factor for periods
    adjust_e = 0.2;     % Utilization adjustment for non-maximum modes

    % Generate utilizations using UUniFast
    util = uunifast(n, total_U);
    
    taskset = cell(1, n);
    for i = 1:n
        % Base period for mode 1 (log-uniform in [10, 1000])
        base_T = 10^(min_T_log10 + (max_T_log10 - min_T_log10) * rand());
        
        % Choose random mode for maximum utilization
        max_util_mode = randi(num_modes);
        
        % Compute base_C for mode 1, but will scale and adjust
        base_C = util(i) * base_T;  % Initial for mode 1
        
        modes = zeros(num_modes, 3);
        for m = 1:num_modes
            scale = scale_f^(m-1);
            T_m = base_T * scale;
            C_m = base_C * scale;
            if m ~= max_util_mode
                C_m = C_m * (1 - adjust_e * rand());  % Uniform [1-adjust_e, 1]
            end
            D_m = T_m;  % Implicit deadline
            modes(m, :) = [C_m, D_m, T_m];
        end
        taskset{i}.modes = modes;
    end
end

% Each row is a C, D, T

function u = uunifast(n, U)
    % UUniFast algorithm to generate n utilizations summing to U
    u = zeros(1, n);
    sum_u = U;
    for i = 1:n-1
        next_sum_u = sum_u * rand()^(1 / (n - i));
        u(i) = sum_u - next_sum_u;
        sum_u = next_sum_u;
    end
    u(n) = sum_u;
end