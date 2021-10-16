function [alpha, timelights, locatelights] = mapdata()
% P=gpxread('gpxfil');
% 
% lat = P.Latitude;
% 
% long = P.Longitude;
% 
% alpha = (lat(:,2:end) - lat(:,1:end-1))./(long(:,2:end) - long(:,1:end-1)); % gradient deltaH/deltaL

    alpha = [];
    timelights = [];
    locatelights = [];
    interval = 10;
    totlongitude = 4000;    % 4 sections, each 1 km
    route = zeros(2, totlongitude/interval + 1);   %first row: latitude; second row: longitude
    uphill_alpha = 0.12;
    downhill_alpha = -0.1;

    for i = 1:1:401
        route(2,i) = interval * (i-1);
        if(i>=301)
            route(1,i) = uphill_alpha * 1000 + downhill_alpha * (i-301) * interval;  %downhill
        elseif(i>=201)
            route(1,i) = uphill_alpha * 1000;   %flat
        elseif(i>=101)
            route(1,i) = uphill_alpha * (i-101) * interval;   %uphill
        end
    end

    traficlights = zeros(1, 401);
    %redlights time + 1000
    traficlights(1, 101) = 1045;
    traficlights(1, 121) = 1030;
    traficlights(1, 231) = 1020;
    traficlights(1, 381) = 1015;
    %greenlights time + 2000
    traficlights(1, 61) = 2030;
    traficlights(1, 171) = 2010;
    traficlights(1, 261) = 2040;
    traficlights(1, 321) = 2025;
    
%     %redlights time + 1000
%     traficlights(1, 101) = 1045;
%     traficlights(1, 121) = 1040;
%     traficlights(1, 231) = 1020;
%     traficlights(1, 381) = 1015;
%     %greenlights time + 2000
%     traficlights(1, 61) = 2030;
%     traficlights(1, 171) = 2060;
%     traficlights(1, 261) = 2040;
%     traficlights(1, 321) = 2025;

    [n,locatelights,timelights] = find(traficlights);

%     locatelights = [locatelights];
    % ans = int8(rem((timelights(1)/1000),2))
    alpha = (route(1,2:end)-route(1,1:end-1))/interval;
end
