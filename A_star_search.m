function path = A_star_search(map, MAX_X, MAX_Y) % Return path

%%
    %This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    distance_type = 1;  % 1 for Manhattan distance
                        % else for Euclidean distance
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    
    MAP(xval,yval)=0;   % set the target on MAP as '0'
    
    %Initialize MAP with location of the obstacle
    % these operations are on the obstacle coordinate
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;  % in MAP set obstacle coordinates as '-1'
    end 
    
    %Initialize MAP with location of the start point
    xval = floor(map(1, 1)) + X_offset;
    yval = floor(map(1, 2)) + Y_offset;
    xStart = xval;
    yStart = yval;
    MAP(xval,yval)=1;   % set the start in MAP as '1'

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list, so they will not be searched
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT = size(CLOSED,1);
    
    %set the starting node as the first node
    %   for now, 'xval' and 'yval' are of the coordinate of start
    xNode=xval;     % 'xNode' is the x component of the coordinate 
    yNode=yval;
    OPEN_COUNT=1;
    
    % find the distance 
    goal_distance = usr_distance(xNode,yNode,xTarget,yTarget,distance_type);
%     goal_distance = distance(xNode,yNode,xTarget,yTarget);

    path_cost=0;    % 'path_cost' is 'h(n)'
    
    % add 
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    Temp_path = [];
    Temp_path(NoPath,1) = xNode;   % record temperary path
    Temp_path(NoPath,2) = yNode;
    
%     %   build a temperary list for motions in different directions
%     motion_list = [];

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while(0) %you have to dicide the Conditions for while loop exit 
        
        % evaluate the first point, (go right)
        x_moved_1 = xval + 1;
        y_moved_1 = yval;
        if ((x_moved_1>MAX_X)||(x_moved_1<1)||(y_moved_1>MAX_Y)||(y_moved_1<1)...
                ||(not(ismember(x_moved_1,CLOSED(:,1))))||...
                (not(ismember(y_moved_1,CLOSED(:,2)))))
            x_moved_1 = x_moved_1 - 1;
        else
            hn_1 = usr_distance(x_moved_1,y_moved_1,xTarget,yTarget,0);
            gn_1 = usr_distance(x_moved_1,y_moved_1,xTarget,yTarget,distance_type);
            fn_1 = hn_1 + gn_1;
            par_x_1 = xval;
            par_y_1 = yval;
        end
        
        % evaluate the second point, (go up right)
        x_moved_2 = xval + 1;
        y_moved_2 = yval + 1;
        if ((x_moved_2>MAX_X)||(x_moved_2<1)||(y_moved_2>MAX_Y)||(y_moved_2<1)...
                ||(not(ismember(x_moved_2,CLOSED(:,1))))||...
                (not(ismember(y_moved_2,CLOSED(:,2)))))
            x_moved_2 = x_moved_2 - 1;
            y_moved_2 = y_moved_2 - 1;
        else
            hn_2 = usr_distance(x_moved_2,y_moved_2,xTarget,yTarget,0);
            gn_2 = usr_distance(x_moved_2,y_moved_2,xTarget,yTarget,distance_type);
            fn_2 = hn_2 + gn_2;
            par_x_2 = xval;
            par_y_2 = yval;
        end
        
        % evaluate the second point, (go up)
        x_moved_3 = xval;
        y_moved_3 = yval + 1;
        if ((x_moved_3>MAX_X)||(x_moved_3<1)||(y_moved_3>MAX_Y)||(y_moved_3<1)...
                ||(not(ismember(x_moved_3,CLOSED(:,1))))||...
                (not(ismember(y_moved_3,CLOSED(:,2)))))
            x_moved_3 = x_moved_3;
            y_moved_3 = y_moved_3 - 1;
        else
            hn_3 = usr_distance(x_moved_3,y_moved_3,xTarget,yTarget,0);
            gn_3 = usr_distance(x_moved_3,y_moved_3,xTarget,yTarget,distance_type);
            fn_3 = hn_3 + gn_3;
            par_x_3 = xval;
            par_y_3 = yval;
        end
        
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node. This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
   path = [];
end
