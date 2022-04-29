 function map = obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y)
%This function returns a map contains random distribution obstacles.
    rand_map = rand(MAX_X,MAX_Y);
    map = [];
    
    % The first colum would be the starting coordinate
    map(1,1) = xStart;
    map(1,2) = yStart;
    
    % afterwards, there are the obstacle coordinates
    k=2;
    obstacle_ratio = 0.25;
    for i = 10:1:23
        for j = 12:1:20
            map(k,1) = i;
            map(k,2) = j;
            k = k + 1;
%             if( (rand_map(i,j) < obstacle_ratio) ...
%                     && (i~= xStart || j~=yStart) ...
%                     && (i~= xTarget || j~=yTarget))
%                 map(k,1) = i;
%                 map(k,2) = j;
%                 k=k+1;
%             end    

        end
    end
    
    for i = 3:1:6
        for j = 6:1:8
            map(k,1) = i;
            map(k,2) = j;
            k = k + 1;
        end
    end
    
    for i = 25:1:28
        for j = 6:1:8
            map(k,1) = i;
            map(k,2) = j;
            k = k + 1;
        end
    end
    
    % in the end (final raw), there are the target coordinate
    map(k,1) = xTarget;
    map(k,2) = yTarget;
end

