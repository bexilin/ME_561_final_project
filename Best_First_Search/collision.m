function z = collision(x,y)

% % inputs: x,y -> position of a point
% % output: z -> z = 0 if the point is inside search space, otherwise z = 1 

global boundary
global boundary_obs
global boundary_front
global boundary_back
global boundary_entry

if isempty(boundary_obs)
    if inpolygon(x,y,boundary(1,:),boundary(2,:)) || ...
            inpolygon(x,y,boundary_entry(1,:),boundary_entry(2,:))
        z = 0;
    else
        z = 1;
    end
else
    if isempty(boundary_entry)
        if inpolygon(x,y,boundary_back(1,:),boundary_back(2,:)) || ...
            inpolygon(x,y,boundary_obs(1,:),boundary_obs(2,:))
            z = 0;
        else
            z = 1;
        end
    elseif isempty(boundary_front)
        if inpolygon(x,y,boundary_back(1,:),boundary_back(2,:)) || ...
            inpolygon(x,y,boundary_obs(1,:),boundary_obs(2,:)) || ...
            inpolygon(x,y,boundary_entry(1,:),boundary_entry(2,:))
            z = 0;
        else 
            z = 1;
        end
    elseif isempty(boundary_back)
        if inpolygon(x,y,boundary_front(1,:),boundary_front(2,:)) || ...
            inpolygon(x,y,boundary_obs(1,:),boundary_obs(2,:)) || ...
            inpolygon(x,y,boundary_entry(1,:),boundary_entry(2,:))
            z = 0;
        else
            z = 1;
        end
    else
        if inpolygon(x,y,boundary_front(1,:),boundary_front(2,:)) || ...
            inpolygon(x,y,boundary_back(1,:),boundary_back(2,:)) || ...
            inpolygon(x,y,boundary_obs(1,:),boundary_obs(2,:)) || ...
            inpolygon(x,y,boundary_entry(1,:),boundary_entry(2,:))
            z = 0;
        else
            z = 1;
        end
    end
end

end