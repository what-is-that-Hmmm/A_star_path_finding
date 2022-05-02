 function path = A_star_search(map, MAX_X, MAX_Y,greedy_h,greedy_g) % Return path

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
    goal_distance = distance(xNode,yNode,xTarget,yTarget);

    path_cost=0;    % 'path_cost' is 'h(n)'
    
    % add the first node to OPEN list
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    i_min = 1;  % initialise the minumum value of the row number
    obs_nodes = CLOSED; % this array containing the obstacles
    gn = OPEN(OPEN_COUNT,7);
    loop_time = 0;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%   'xval' & 'yval' stores the coordinate of parent node
%   'gn' is the accumulative cost of parent node
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    while((xval~=xTarget)||(yval~=yTarget)) %you have to dicide the Conditions for while loop exit  
        node_x = xval;
        node_y = yval;
        
        if (i_min==-1 && repeated ==0)
            error('no path found')
            break
        end
        
        new_nodes = expand_array(node_x,node_y,gn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y,greedy_h,greedy_g);
        %'new_nodes' FORMAT
        %greedy_h for biasing on h(n), agent tend to go to target
        %greedt_g for biasing on g(n), agent tend to explore
        %--------------------------------
        %|X val |Y val | h(n) |g(n)|f(n)|
        %--------------------------------

        size_new_nodes = size(new_nodes,1);
        
        % Checking repeated items
        for i2=1:(CLOSED_COUNT-1) 
            if (CLOSED(i2,1) ==xval && CLOSED(i2,2) == yval)
                repeated = 1; % its in the CLOSED
                break
            else
                repeated = 0;   % its not of CLOSED
            end
        end     % end the for loop: Checking repeated items
        
        OPEN(i_min,1) = 0;     % remove the node from OPEN
        CLOSED(CLOSED_COUNT,1)=xval;% add the expanded node to CLOSED     
        CLOSED(CLOSED_COUNT,2)=yval;
        CLOSED_COUNT = CLOSED_COUNT+1;

        if (repeated == 0 || loop_time == 0 )
            for ii=1:size_new_nodes     % add expanded nodes to OPEN list
                row_num = OPEN_COUNT+ii;
                OPEN(row_num,1) = 1;  % mark new nodes as an open node
                OPEN(row_num,2:3) = new_nodes(ii,1:2);  % add coorinates to OPEN
                OPEN(row_num,4:5) = [xval, yval];       % add parent coordinates to OPEN
                OPEN(row_num,6:8) = new_nodes(ii,3:5);  % add costs to OPEN
            end     % end of the for loop: 'add expanded nodes to OPEN list'
        end
        %   Find the next node to be expanded from OPEN
        OPEN_COUNT = size(OPEN,1);
        i_ref = CLOSED_COUNT -1;    % preseve final node index
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);    % i_min indicates the row number.
        if (i_min~=-1)  % explore the node with minum f(n)
            xval = OPEN(i_min,2);
            yval = OPEN(i_min,3);
            gn = OPEN(i_min, 7);
        else
            i_final = i_ref;
        end     % end of the: 'explore the node with minum f(n)'
        
        % visualise the process
        scatter(OPEN(:,2),OPEN(:,3),'g')
        hold on
        scatter(CLOSED(:,1),CLOSED(:,2),'filled','d')
        scatter(xval,yval,'b','filled')
        hold off
        loop_time = loop_time + 1;
        
     
    end %End of While Loop
    
    % Once algorithm has run, The optimal path is generated by starting of at the
    % last node(if it is the target node) and then identifying its parent node
    % until it reaches the start node. This is the optimal path
    
    % get the optimal path after searching
    path = [];
    path(1,1)=xTarget;
    path(1,2)=yTarget;   % path starts at the start
    xNext = 0;
    yNext = 0;
    empty_node = 0;
    
    OPEN_length = size(OPEN,1);
    for i3 = OPEN_length:-1:1   % finding the line index lead to target
        if (OPEN(i3,2)==xTarget && OPEN(i3,3)==yTarget && NoPath == 1)
            path(NoPath,:) = [xTarget, yTarget];
            NoPath = NoPath + 1;
            xNext = OPEN(i3,4);
            yNext = OPEN(i3,5);
        end
        
        if(OPEN(i3,2)==xNext && OPEN(i3,3)==yNext && NoPath ~= 1)
            path(NoPath,:) = [xNext, yNext];
            NoPath = NoPath + 1;
            xNext = OPEN(i3, 4);
            yNext = OPEN(i3, 5);
        else
            empty_node = empty_node + 1;
        end
    end  %  end of the for loop
    
%     path(1,:) = [xTarget,yTarget];  
%     path(2,:) = CLOSED(i_ref,:);
%     x_ref = CLOSED(i_ref,1);
%     y_ref = CLOSED(i_ref,2);
end
