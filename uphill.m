function [T, W, V] = uphill(alpha, distance, v1, v_const, acc)
%UPHILL Compute time T, energy consumption W and final speed V2 when on a
%uphill road. (alpha > 0)(acc < 0)
%   [T, W, V] = UPHILL(alpha, distance, v1, v_const, acc) computes the time
%   cost T, energy consuption W and final speed V on this uphill road section.

global m M g fr air_density Cx area
T = 0; 
W = 0;
V = 0;
Actual_dis = distance / cos(alpha);

P_constV = 0.5*air_density*Cx*area*v_const^3 + (m+M)*g*(fr*cos(alpha)+sin(alpha))*v_const;
W_dec_fun = @(t)((m+M)*acc+((m+M)*g*(fr*cos(alpha)+sin(alpha)))+0.5*air_density*Cx*area.*(v1+acc.*t).^2).*(v1+acc.*t);
W_acc_fun = @(t)((m+M)*acc+((m+M)*g*(fr*cos(alpha)+sin(alpha)))+0.5*air_density*Cx*area.*(v1-acc.*t).^2).*(v1-acc.*t);


if (v1 >= v_const)
    x_dec = (v_const^2 - v1^2) / (2*acc);    %deceleration distance from v1 to constant speed
    if(x_dec == 0)
        %Steady state, constant speed cycling
        V = v_const;
        T = Actual_dis / V;
        W = T * P_constV;
    elseif(x_dec <= Actual_dis)
        %Enough distance for completing the deceleration phase
        %Constant decleration and then constant speed
        t_dec = (v_const - v1) / acc;
        x_const = Actual_dis - x_dec;       
        t_const = x_const / v_const;     
        T = t_dec + t_const;
        W_dec = integral(W_dec_fun, 0, t_dec);
        W_const = t_const * P_constV;
        W = W_dec + W_const;
        V = v_const;
    else
        %Constant deceleration
        V = sqrt(v1^2 + 2*acc*Actual_dis);
        T = (V - v1) / acc;
        W = integral(W_dec_fun, 0, T);
    end
else
    x_acc = (v1^2 - v_const^2) / (2*acc);
    if(x_acc <= Actual_dis)
        %Enough distance for completing the acceleration phase
        %Constant accleration and then constant speed
        t_dec = (v1 - v_const) / acc;
        x_const = Actual_dis - x_acc;       
        t_const = x_const / v_const;     
        T = t_dec + t_const;
        W_dec = integral(W_acc_fun, 0, t_dec);
        W_const = t_const * P_constV;
        W = W_dec + W_const;
        V = v_const;
    else
        %Constant acceleration
        V = sqrt(v1^2 - 2*acc*Actual_dis);
        T = (v1 - V) / acc;
        W = integral(W_acc_fun, 0, T);
    end
end

end

