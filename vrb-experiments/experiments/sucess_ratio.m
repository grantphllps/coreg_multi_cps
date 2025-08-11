Uvals  = 0.05 : 0.10 : 0.95;
tests  = {
    @rta_sp, 
    % @vrb_l1, 
    % @vrb_l2, 
    % @vrb_ilp,
    % @vrb_ilp_con, 
    % @ub_n, 
    % @ub_nx
    };
Sratio = zeros(length(Uvals), length(tests));

for u = 1:length(Uvals)
    for rep = 1:1000                 % 1000 task-sets / U
        TS = generate_vrb_tasksets(Uvals(u),10,0.5,5, rep);
        for t = 1:length(tests)
            Sratio(u,t) = opa(TS, tests{t});
        end
    end
end
Sratio = Sratio / 1000;              % convert to probability
plot(Uvals, Sratio)