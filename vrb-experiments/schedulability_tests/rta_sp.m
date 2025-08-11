function ok = rta_sp(taskset)
% Build a pure-sporadic clone with Cmax, Tmin, Dmin for each VRB
for t = taskset
   if isfield(t,'VRB')
       t.C = max( [t.VRB.C] );
       t.T = min( [t.VRB.T] );
       t.D = min( [t.VRB.D] );
       t = rmfield(t,'VRB');
   end
   sp_set(end+1) = t; %#ok<AGROW>
end
ok = rta_standard(sp_set);   % textbook FP response-time test
end