function ts = generate_vrb_tasksets(U_tot, n, p_vrb, num_modes, rng_seed)
% Exact parameters in §V-A (numbers in brackets are the paper’s notation):
%   • log-uniform period spread:           r = 2          (10^r range)
%   • mode scaling factor:                 f = 1.5
%   • WCET variability within a task:      e = 0.25
%   • deadline scale (constrained):        x = 0.5
%   • % of VRB tasks:                      p (here p_vrb)
%
% Every call produces ONE task-set.  The caller loops over utilisation.

if nargin>4, rng(rng_seed); end
% --- Step 1 Utilisation split ---------------------------
U = uunifast(U_tot, n);             % algorithm of Bini & Buttazzo [10]

% --- Step 2 generate base sporadic tasks ---------------
r  = 2;            f = 1.5;
e  = 0.25;         x = 0.5;
Ti_min = 10;
Ti_max = Ti_min * 10^r;

for i = 1:n
    T  = log_uniform(Ti_min, Ti_max);
    C  = U(i)*T;
    D  = rand()*((T - x*C) - C) + C;   % constrained deadline
    tasks(i) = struct('T',T,'C',C,'D',D); %#ok<AGROW>
end

% --- Step 3 promote p% to VRB --------------------------
masks           = false(1,n);
masks(randperm(n, round(p_vrb*n))) = true;
T1max = max([tasks(masks).T]);

for i = find(masks)
    modes(1).T = tasks(i).T;
    modes(1).C = tasks(i).C;
    modes(1).D = tasks(i).D;
    for m = 2:num_modes
        modes(m).T = tasks(i).T * f^(m-1);
        modes(m).C = tasks(i).C * (1 - rand()*e);
        % implicit OR constrained randomly:
        modes(m).D = rand()*((modes(m).T - x*modes(m).C) - modes(m).C) + ...
                     modes(m).C;
    end
    tasks(i).VRB = modes;
end
ts = tasks;
end
