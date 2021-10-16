function [T, W, V] = flat(distance, v1, v_const, acc)
%FLAT Compute time T, energy consumption W and final speed V2 when on a
%flat road. (alpha = 0)(acc > 0)
%   [T, W, V] = FLAT(distance, v1, v_const, acc) computes the value of
%   time cost T, energy consuption W and final speed V on this flat road
%   section.

global m M g fr air_density Cx area
T = 0; 
W = 0;
V = 0;

P_constV = 0.5*air_density*Cx*area*v_const^3 + (m+M)*g*fr*v_const;
W_acc_fun = @(t)((m+M)*acc+(m+M)*g*fr+0.5*air_density*Cx*area.*(v1 + acc.*t).^2).*(v1 + acc.*t);

x_acc = (v_const^2 - v1^2) / (2*acc);    %acceleration distance from v1 to constant speed
if(x_acc == 0)
    %Steady state, constant speed cycling
    V = v_const;
    T = distance / V;
    W = T * P_constV;
elseif(x_acc <= distance)
    %Enough distance for completing the acceleration phase
    %Constant accleration and then constant speed
    t_acc = (v_const - v1) / acc;
    x_const = distance - x_acc;       
    t_const = x_const / v_const;     
    T = t_acc + t_const;
    V = v_const;
    W_acc = integral(W_acc_fun, 0, t_acc);
    W_const = t_const * P_constV;
    W = W_acc + W_const;
else
    %Constant acceleration
    V = sqrt(v1^2 + 2*acc*distance);    %v2^2-v1^2 = 2*accleration*x
    T = (V - v1) / acc;
    W = integral(W_acc_fun, 0, T);
end 
end

