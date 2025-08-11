function I = linear_bounds(task, w, choice)
% choice==1  implements Eq. (6);  choice==2  Eq. (7)
Cmax = max([task.VRB.C]);
Umax = max([task.VRB.C] ./ [task.VRB.T]);
switch choice
  case 1, I = Umax*w + Cmax;
  case 2
       t1 = Cmax / Umax;
       I = Umax*w + Cmax - max(0, Umax*(w-t1));
end
I = floor(I);                % Eq. (8)
end