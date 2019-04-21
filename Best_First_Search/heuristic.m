function path = heuristic(start,goal)

% Node vector components:
% x,y,theta,self_id,parent_id,[x1,y1,theta1,...,x4,y4,theta4]
start_node = [start;zeros(15,1)];

% initialize parameters
u = 20;
L = 2.5;
dt = 0.05;
input = [-pi/10,-pi/22,0,pi/22,pi/10];
max_id = 1;
path = [];

% initialize frontier and explored list
frontier = start_node;
explored = zeros(18,0);

while ~isempty(frontier)
    % get current node from frontier
    [~,I] = min(frontier(6,:));
    current_node = frontier(:,I);
    frontier(:,I) = [];
    
    % add to explored if not exist
    if isempty(explored)
        %plot(current(1),current(2),'rx')
        %hold on
        max_id = max_id +1;
        current_node(4) = max_id;
        explored = current_node;
    elseif min(vecnorm(explored(1:3,:)-current_node(1:3)))>0.005
        %plot(current(1),current(2),'rx')
        %hold on
        max_id = max_id +1;
        current_node(4) = max_id;
        explored = [explored current_node];
    else
        continue
    end
    
    % If found the goal, return the path
    if norm(current_node(1:2)-goal(1:2))<5
        current_id = current_node(4);
        while current_id ~= 0
            m = find(explored(4,:)==current_id);
            path_append = [reshape(explored(7:18,m),3,[]) explored(1:3,m)];
            path = [path_append path];
            current_id = explored(5,m);
        end
        path(:,1:4)=[];
        return
    end
            
    % Generate children
    for i = input
        
        % compute new node postion by doing Euler integration for 10 times
        x = current_node(1);
        y = current_node(2);
        theta = current_node(3);
        thetadot = u/L*tan(i);
        sequence = zeros(12,1);
        for j = 1:5
            theta = theta + dt*thetadot;
            x = x + dt*u*cos(theta);
            y = y + dt*u*sin(theta);
            if j < 5
                sequence(3*j-2:3*j) = [x y theta]';
            end
            %plot(x,y,'gx')
            %hold on
        end
        
        % check collision
        if collision(x,y)
            %plot(x,y,'bx')
            %hold on
            continue
        else
            child = [x,y,theta,zeros(1,15)]';
            child(6) = norm([x y]'-goal(1:2));
            child(5) = current_node(4);
            child(7:18) = sequence;
            frontier = [frontier child];
        end
    end
    
end

return;

end