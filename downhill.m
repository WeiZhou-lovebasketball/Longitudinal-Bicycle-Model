function [T, W, V] = downhill(alpha, distance, v1, v_const, acc)
%DOWNHILL Compute time T, energy consumption W and final speed V2 when on a
%downhill road. (alpha < 0)(acc > 0)
%   [T, W, V] = DOWNHILL(alpha, distance, v1, v_const, acc) computes the time cost
%   T, energy consuption W and final speed V on this downhill road section.

%v_const here should be the speed limit.

T = 0; 
W = 0;
V = 0;
Actual_dis = distance / cos(alpha);

x_acc = (v_const^2 - v1^2) / (2*acc);    %acceleration distance from v1 to constant speed
if(x_acc == 0)
    %Steady state, constant speed cycling
    V = v_const;
    T = Actual_dis / V;
    %Applying brake, no energy consumption
elseif(x_acc <= Actual_dis)
    %Enough distance for completing the acceleration phase
    %Constant accleration and then constant speed
    t_acc = (v_const - v1) / acc;
    x_const = Actual_dis - x_acc;       
    t_const = x_const / v_const;     
    T = t_acc + t_const;
    V = v_const;
    %Gravity and brake do the job, no energy consumption
else
    %Constant acceleration, still no energy consumption
    V = sqrt(v1^2 + 2*acc*Actual_dis);
    T = (V - v1) / acc;
end

