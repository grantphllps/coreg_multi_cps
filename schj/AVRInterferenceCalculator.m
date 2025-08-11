classdef AVRInterferenceCalculator < handle
    properties
        boundaries
        Cs
        alpha_minus
        alpha_plus
        delta_theta
        memo % containers.Map
    end

    methods
        function obj = AVRInterferenceCalculator(modes, max_period_ms, alpha_minus, alpha_plus, delta_theta)
            if nargin < 5
                delta_theta = 60000.0;
            end
            % Assume modes is Nx2 matrix: column 1 min_period_ms, column 2 C
            period_boundaries_ms = [modes(:,1)', max_period_ms];
            obj.Cs = modes(:,2)';
            omegas = delta_theta ./ period_boundaries_ms;
            obj.boundaries = sort(omegas, 'descend');
            obj.alpha_minus = alpha_minus;
            obj.alpha_plus = alpha_plus;
            obj.delta_theta = delta_theta;
            obj.memo = containers.Map('KeyType', 'char', 'ValueType', 'double');
        end

        function C = get_C(obj, omega)
            if omega > obj.boundaries(1)
                C = obj.Cs(1);
                return
            end
            if omega < obj.boundaries(end)
                C = obj.Cs(end);
                return
            end
            for i = 1:length(obj.Cs)
                if obj.boundaries(i) >= omega && omega >= obj.boundaries(i+1)
                    C = obj.Cs(i);
                    return
                end
            end
            error('Omega %f out of bounds', omega);
        end

        function t = T_func(obj, omega, alpha)
            if alpha == 0
                t = obj.delta_theta / omega;
                return
            end
            disc = omega^2 + 2 * alpha * obj.delta_theta;
            if disc < 0
                t = Inf;
                return
            end
            sqrt_disc = sqrt(disc);
            t = (sqrt_disc - omega) / alpha;
            if t <= 0
                t = Inf;
            end
        end

        function next_omega = Omega_func(obj, omega, alpha)
            disc = omega^2 + 2 * alpha * obj.delta_theta;
            if disc < 0
                next_omega = [];
                return
            end
            next_omega = sqrt(disc);
        end

        function I_omega = compute_I_omega(obj, omega, t)
            obj.memo = containers.Map('KeyType', 'char', 'ValueType', 'double'); % Clear by reinitializing
            I_omega = obj.max_workload(0.0, omega, t);
        end

        function workload = max_workload(obj, current_time, omega, t)
            if current_time >= t
                workload = 0;
                return
            end
            key = sprintf('%.6f_%.6f', current_time, omega);
            if isKey(obj.memo, key)
                workload = obj.memo(key);
                return
            end
            C = obj.get_C(omega);
            min_T = obj.T_func(omega, obj.alpha_plus);
            max_T = obj.T_func(omega, obj.alpha_minus);
            possible_T = [];
            if min_T < Inf
                possible_T = [possible_T, min_T];
            end
            min_next_omega = obj.Omega_func(omega, obj.alpha_minus);
            max_next_omega = obj.Omega_func(omega, obj.alpha_plus);
            if isempty(min_next_omega) || isempty(max_next_omega)
                workload = C;
                obj.memo(key) = workload;
                return
            end
            for ii = 1:length(obj.boundaries)
                b = obj.boundaries(ii);
                if min_next_omega <= b && b <= max_next_omega && b ~= omega
                    a_b = (b^2 - omega^2) / (2 * obj.delta_theta);
                    if obj.alpha_minus <= a_b && a_b <= obj.alpha_plus
                        T_b = 2 * obj.delta_theta / (omega + b);
                        if T_b > 0
                            possible_T = [possible_T, T_b];
                        end
                    end
                end
            end
            if max_T < Inf
                possible_T = [possible_T, max_T];
            end
            possible_T = unique(possible_T(possible_T > 0 & possible_T < Inf));
            possible_T = sort(possible_T);
            max_next_w = 0;
            for ii = 1:length(possible_T)
                next_T = possible_T(ii);
                next_omega = 2 * obj.delta_theta / next_T - omega;
                next_time = current_time + next_T;
                if next_time < t
                    next_w = obj.max_workload(next_time, next_omega, t);
                    if next_w > max_next_w
                        max_next_w = next_w;
                    end
                end
            end
            workload = C + max_next_w;
            obj.memo(key) = workload;
        end

        function max_I = worst_case_interference(obj, t)
            max_I = 0;
            for ii = 1:length(obj.boundaries)
                omega = obj.boundaries(ii);
                I_omega = obj.compute_I_omega(omega, t);
                if I_omega > max_I
                    max_I = I_omega;
                end
            end
        end
    end
end