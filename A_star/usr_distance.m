function dist = usr_distance(x1,y1,x2,y2,dis_type)
%This function calculates the distance between any two cartesian 
%coordinates.
%   distance can be calculated based on different distance type
if(dis_type == 1)
    dist = abs(x1-x2)+abs(y1-y2);
else
    dist=floor(sqrt((x1-x2)^2 + (y1-y2)^2));    % use 'floor' to take the smaller integer 
end
end

