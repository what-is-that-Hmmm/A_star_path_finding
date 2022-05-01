function dist = usr_distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   x1 & y1 are the coordinates of the start coordinates
%   x2 & y2 are the coordinates of the end coordinates
%   distance can be calculated based on different distance type

    dist = floor(sqrt((x1-x2)^2 + (y1-y2)^2)); %abs(x1-x2)+abs(y1-y2);

end

