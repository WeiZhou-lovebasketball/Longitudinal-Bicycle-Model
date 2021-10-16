clear all
clc
 
global Pmax m M air_density area Cx g fr
Pmax = 150; %W, maximum power when cycling (for boosting in acceleration phase)
P = 125; %Steady state power
m = 80;  %kg, mass of the passenger
M = 20;  %kg, mass of the bicycle
% h =  %m, height of the cyclist
% BMI =  % BMI factor to identify weak or strong cyclist
air_density = 1.23;  %
area = 0.5;  %
Cx = 0.4;  %
g = 9.81;  
fr = 0.012;  %road friction factor
bio_efficiency = 0.25;
% mechanical_efficency = ;
a = 0.25;   %deceleration/acceleration rate, m/s^2

[alpha, lighttime, lightlocation] = mapdata();    % road inclination from map data
interval = 10;
n = size(alpha, 2);
Tstore = zeros(1, n);
Wstore = zeros(1, n);
Vstore = zeros(1, n+1);
N = 3;      %Horizon, 30 m before the traffic light
lightlocation = [N+1, lightlocation];
nlights = length(lightlocation)-1;
StopandGo = 0;   %1 = stop, 0 = stop

syms v
eqn = 0.5*air_density*Cx*area*v^3 + (m+M)*g*fr*v - P == 0;
roots1 = double(solve(eqn, v, 'MaxDegree', 3));
v_flat_const = roots1(1);

v_limit = 10;    %speed limit

for i = 1:N
    if (alpha(1, i) == 0)
        %Flat road
        [T, W, V] = flat(interval, Vstore(i), v_flat_const, a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    elseif (alpha(1, i)>0)
        %Uphill
        syms v
        eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,i))+sin(alpha(1,i)))*v - P == 0;
        roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
        v_up_const = roots2(1);
        [T, W, V] = uphill(alpha(1,i), interval, Vstore(i), v_up_const, -a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    else 
        %Downhill
        [T, W, V] = downhill(alpha(1,i), interval, Vstore(i), v_limit, a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    end
end 

for j = 1:nlights
    for i = lightlocation(j):lightlocation(j+1)-1
        if (alpha(1, i) == 0) 
            [T, W, V] = flat(interval, Vstore(i), v_flat_const, a);
            Vstore(1, i+1) = V;
            Tstore(1, i)= T;
            Wstore(1, i) = W;
        elseif (alpha(1, i)>0)
            %Uphill
            syms v
            eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,i))+sin(alpha(1,i)))*v - P == 0;
            roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
            v_up_const = roots2(1);
            [T, W, V] = uphill(alpha(1,i), interval, Vstore(i), v_up_const, -a);
            Vstore(1, i+1) = V;
            Tstore(1, i)= T;
            Wstore(1, i) = W;
        else 
            %Downhill
            [T, W, V] = downhill(alpha(1,i), interval, Vstore(i), v_limit, a);
            Vstore(1, i+1) = V;
            Tstore(1, i)= T;
            Wstore(1, i) = W;
        end
    end
    
    %Detect the traffic light and return a stop signal.(stopandgo = 1)
    if (int8(rem((lighttime(j)/1000),2)) == 1)
        %Stop sign/ Red light!
        if (lighttime(j)-1000 <= (sum(Tstore(i-2 : i))))
            % Redlight time <= time needed to arrive at the light
            % Keep going!
            StopandGo = 0;
        else
            % Redlight time > time needed to arrive at the light
            % Decelerate and Stop!   
            StopandGo = 1;
        end
    elseif (int8(rem((lighttime(j)/1000),2)) == 0)
        %No stop sign/ Green light, keep going!
        if (lighttime(j)-2000 <= (sum(Tstore(i-2 : i))))
            % Greenlight time <= time needed to arrive at the light
            % Decelerate and Stop! 
            StopandGo = 1;
        else
            % Redlight time > time needed to arrive at the light
            % Keep going!   
            StopandGo = 0;
        end
    end
        
    if (StopandGo == 1)
        Vstop = Vstore(i-2: i);
        brake = Vstop(1)^2 / (2 * N * interval);     %deceleration rate, 30 m from v to 0
        Tbrake = Vstop(1) / brake;
        Vstop(2) = sqrt(Vstop(1)^2 - 2 * interval * brake);
        tbrake1 = (Vstop(1) - Vstop(2)) / brake;
        Vstop(3) = sqrt(Vstop(2)^2 - 2  * interval * brake);
        tbrake2 = (Vstop(2) - Vstop(3)) / brake;
        tbrake3 = Vstop(3) / brake;
        xbrake = ones(1, n) * interval;
        vswitch = zeros(1, n+1);
        
        if (int8(rem((lighttime(j)/1000),2)) == true)
            %Red light, brake, but the travelling time of 30 m will differ
            %from previous one, so it has two situations: 1.light time >=
            %Time for brake, we need to wait at the light. 2. light time < 
            %Time for brake, the light turns green when we are decelerating, 
            %we need to speed up again.
            %Brake with constant deceleration rate
            if (lighttime(j)-1000 >= Tbrake)
                %Situation 1
                %The third brake time is stored toghther with the
                %waiting time
                Vstore(i-2:i) = Vstop(1: 3);
                Vstore(i+1) = 0;
                Tstore(i-2: i) = [tbrake1, tbrake2, lighttime(j)-1000-tbrake1-tbrake2];
                %Apply brake, no energy consumption
                Wstore(i-2: i) = 0;
            elseif (lighttime(j)-1000 >= tbrake1 + tbrake2)
                %Situation 2
                %Switch point(red->green) in the third interval
                %Recalculate the final interval alpha(i)
                Vstore(i-2:i) = Vstop(1: 3);
                tswitch = lighttime(j)- 1000 - tbrake1 - tbrake2;
                Vswitch = Vstop(3) - brake * tswitch;
                x = interval - 0.5 * (Vswitch + Vstop(3)) * tswitch;
                if (alpha(1, i) == 0)
                    [Tacc, Wacc, Vacc] = flat(x, Vswitch, v_flat_const, a);
                elseif (alpha(1, i) > 0)
                    syms v
                    eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,i))+sin(alpha(1,i)))*v - P == 0;
                    roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
                    v_up_const = roots2(1);
                    [Tacc, Wacc, Vacc] = uphill(alpha(1, i), x, Vswitch, v_up_const, -a);
                else
                    [Tacc, Wacc, Vacc] = downhill(alpha(1,i), x, Vswitch, v_limit, a);
                end 
                Vstore(i+1) = Vacc;
                Wstore(i-2: i) = [0, 0, Wacc];
                Tstore(i-2: i) = [tbrake1, tbrake2, tswitch + Tacc];
            elseif (lighttime(j)-1000 >= tbrake1)
                %Switch point(red->green) in the second interval
                %Recalculate two intervals alpha(i-1), alpha(i)
                tswitch = lighttime(j)- 1000 - tbrake1;
                Vstore(i-1) = Vstop(2);
                Vswitch = Vstore(i-1) - brake * tswitch;
                x = interval - 0.5 * (Vswitch + Vstore(i-1)) * tswitch;
                xbrake(i-1) = x;
                vswitch(i-1) = Vswitch;
                for k = i-1:i
                    if (alpha(1, k) == 0) 
                        [Tacc, Wacc, Vacc] = flat(xbrake(k), vswitch(k), v_flat_const, a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    elseif (alpha(1, k) > 0)
                        syms v
                        eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,k))+sin(alpha(1,k)))*v - P == 0;
                        roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
                        v_up_const = roots2(1);
                        [Tacc, Wacc, Vacc] = uphill(alpha(1, k), xbrake(k), vswitch(k), v_up_const, -a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    else
                        [Tacc, Wacc, Vacc] = downhill(alpha(1, k), xbrake(k), vswitch(k), v_limit, a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    end
                end
                Vstore(i: i+1) = vswitch(i: i+1);
                Wstore(i-2) = 0;
                Tstore(i-2) = tbrake1;
                Tstore(i) = Tstore(i) - tswitch;
            else
                %Switch point(red->green) in the first interval
                %Recalculate three intervals alpha(i-2), alpha(i-1), alpha(i)
                tswitch = lighttime(j)- 1000;
                Vswitch = Vstore(i-2) - brake * tswitch;
                x = interval - 0.5 * (Vswitch + Vstore(i-2)) * tswitch;
                xbrake(i-2) = x;
                vswitch(i-2) = Vswitch;
                for k = i-2:i
                    if (alpha(1, k) == 0)
                        [Tacc, Wacc, Vacc] = flat(xbrake(k), vswitch(k), v_flat_const, a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    elseif (alpha(1, k) > 0)
                        syms v
                        eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,k))+sin(alpha(1,k)))*v - P == 0;
                        roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
                        v_up_const = roots2(1);
                        [Tacc, Wacc, Vacc] = uphill(alpha(1, k), xbrake(k), vswitch(k), v_up_const, a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    else
                        [Tacc, Wacc, Vacc] = downhill(alpha(1, k), xbrake(k), vswitch(k), v_limit, a);
                        vswitch(k+1) = Vacc;
                        Tstore(k) = Tacc + tswitch;
                        Wstore(k) = Wacc;
                    end
                end
                Vstore(i-1: i+1) = vswitch(i-1: i+1);
                Tstore(i-1: i) = Tstore(i-1: i) - tswitch;
            end
        else
            %Green light, brake and then wait till the next green light.
            %Assume each light lasts for 60 s, 120s as one period.
            Vstore(i-2: i) = Vstop(1:3);
            Vstore(i+1) = 0;
            %Green light turns red and the third phase of time is
            %stored toghther with the waiting time, which is red light
            %= 60s, plus tbrake1+2+3 - green light time.
            Tstore(i-2: i) = [tbrake1, tbrake2, tbrake1+tbrake2+tbrake3+60-(lighttime(j)-2000)];
            Wstore(i-2: i) = 0;
        end
    end
end

for i = lightlocation(end):n
    if (alpha(1, i) == 0)
        %Flat road
        [T, W, V] = flat(interval, Vstore(i), v_flat_const, a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    elseif (alpha(1, i)>0)
        %Uphill
        syms v
        eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,i))+sin(alpha(1,i)))*v - P == 0;
        roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
        v_up_const = roots2(1);
        [T, W, V] = uphill(alpha(1,i), interval, Vstore(i), v_up_const, -a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    else 
        %Downhill
%         v_limit = 10;    %speed limit
        [T, W, V] = downhill(alpha(1,i), interval, Vstore(i), v_limit, a);
        Vstore(1, i+1) = V;
        Tstore(1, i)= T;
        Wstore(1, i) = W;
    end
end 

% for i = 1:n
%     if (alpha(1, i) == 0)
%         %Flat road
%         acc = 0.25;   %acceleration rate
%         syms v
%         eqn = 0.5*air_density*Cx*area*v^3 + (m+M)*g*fr*v - P == 0;
%         roots1 = double(solve(eqn, v, 'MaxDegree', 3));
%         v_const_flat = roots1(1);
%         [T, W, V] = flat(interval, Vstore(i), v_const_flat, acc);
%         Vstore(1, i+1) = V;
%         Tstore(1, i)= T;
%         Wstore(1, i) = W;
%     elseif (alpha(1, i)>0)
%         %Uphill
%         acc = - 0.25;
%         syms v
%         eqn2 = 0.5*air_density*Cx*area*v^3 + (m+M)*g*(fr*cos(alpha(1,i))+sin(alpha(1,i)))*v - P == 0;
%         roots2 = double(solve(eqn2, v, 'MaxDegree', 3));
%         v_const_uphill = roots2(1);
%         [T, W, V] = uphill(alpha(1,i), interval, Vstore(i), v_const_uphill, acc);
%         Vstore(1, i+1) = V;
%         Tstore(1, i)= T;
%         Wstore(1, i) = W;
%     else 
%         %Downhill
%         acc = 0.25;
%         v_limit = 10;    %speed limit
%         [T, W, V] = downhill(alpha(1,i), interval, Vstore(i), v_limit, acc);
%         Vstore(1, i+1) = V;
%         Tstore(1, i)= T;
%         Wstore(1, i) = W;
%     end
% end    

length = [];
for i = 1:n
    length(i) = interval * i;
end

T_total = sum(Tstore);
fprintf('The travel time of estimation:\n'); fprintf('%f s\n', T_total);
W_total = sum(Wstore);
W_total = W_total / bio_efficiency;
fprintf('The energy consumption:\n'); fprintf('%f J\n', W_total);

figure(1)
yyaxis left
plot(length, Tstore,'-o');
title('Travelling time and energy consumption in each interval')
xlabel('Longitude / m');
ylabel('Travelling time / s');
yyaxis right
plot(length, Wstore,'--*');
ylabel('Energy consumption / J');

figure(2)
plot(cat(2,[0],length), Vstore);
title('Speed at each interval');
xlabel('Longitude / m');
ylabel('Speed / m/s');
hold on
plot([600,600],[0,10], 'g--');
plot([1000,1000],[0,10],'r--');
plot([1200,1200],[0,10],'r--');
plot([1700,1700],[0,10],'g--');
plot([2300,2300],[0,10],'r--');
plot([2600,2600],[0,10],'g--');
plot([3200,3200],[0,10],'g--');
plot([3800,3800],[0,10],'r--');
