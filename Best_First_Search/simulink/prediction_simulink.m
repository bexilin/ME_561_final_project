function sequence = prediction_simulink_2(initial,u,dt,number,cline,obstacle,direction,vertical,right,center,left,cline_right,cline_left,obs_cline)

% global cline obstacle direction vertical right center left cline_right cline_left obs_cline 

%% input
required_length = u*dt*number;

%% parameters
cline_to_right = 3;
cline_to_left = 9;
cline_half = 0.65;
section = 300;
entry_boundary = 40;
boundary_scale = 0.2;
entry_margin = 1;
entry_length = 200;
entry_to_obs = 20;
obs_to_front = 100;
obs_to_end = 70;
obs_front = 80;
obs_end = 10;
obs_left = 4;
obs_left_front = 40;


%% compute the section of road for prediction
v_1 = vecnorm(cline-[initial(1)*ones(1,length(cline));initial(2)*ones(1,length(cline))]);
if isempty(v_1)
    I_start = 0;
else
    [~,I_start] = min(v_1);
end

% shift point sets if index out of range
shift = 0;
if I_start+section > length(cline)
    shift = length(cline) - (I_start+section);
    I_start = I_start + shift;
    cline = circshift(cline,shift,2);
    direction = circshift(direction,shift,2);
    vertical = circshift(vertical,shift,2);
    right = circshift(right,shift,2);
    center = circshift(center,shift,2);
    left = circshift(left,shift,2);
    cline_right = circshift(cline_right,shift,2);
    cline_left = circshift(cline_left,shift,2);
    obs_cline = circshift(obs_cline,shift,2);
end

% check if the length of section 
section_length = sum(vecnorm(diff(cline(:,I_start:I_start+section),1,2)));
if section_length < 1.2*required_length 
    section = ceil(section*(1.2*required_length)/section_length);
    if I_start+section > length(cline)
        shift_2 = length(cline) - (I_start+section);
        I_start = I_start + shift_2;
        cline = circshift(cline,shift_2,2);
        direction = circshift(direction,shift_2,2);
        vertical = circshift(vertical,shift_2,2);
        right = circshift(right,shift_2,2);
        center = circshift(center,shift_2,2);
        left = circshift(left,shift_2,2);
        cline_right = circshift(cline_right,shift_2,2);
        cline_left = circshift(cline_left,shift_2,2);
        obs_cline = circshift(obs_cline,shift_2,2);
        shift = shift + shift_2;
    end
end

right_part = right(:,I_start:I_start+section);
center_part = center(:,I_start:I_start+section);
left_part = left(:,I_start:I_start+section);
cline_right_part = cline_right(:,I_start:I_start+section);
cline_left_part = cline_left(:,I_start:I_start+section);

%% search space boundaries after entry part when no obstacle exists in the road section 
% global boundary
boundary = [cline_right_part(:,1+entry_length:end) fliplr(cline_left_part(:,1+entry_length:end)) cline_right_part(:,1+entry_length)];
% plot(cline(1,I_start:I_start+section),cline(2,I_start:I_start+section),'g',right_part(1,:),right_part(2,:),...
%     'k',left_part(1,:),left_part(2,:),'k',center_part(1,:),center_part(2,:),'k--')
% hold on

%% search space boundaries after entry part when an obstacle exists in the road section
% global boundary_obs
% global boundary_front
% global boundary_back
boundary_obs = [];
boundary_front = [];
boundary_back = [];
cline_idx = 0;
v_2 = find(ismember(obs_cline,I_start:I_start+section));
if isempty(v_2)
    obs_idx = 0;
else
    obs_idx = min(v_2);
end
if obs_idx ~= 0 
    cline_idx = obs_cline(obs_idx);
    boundary_obs_1 = left(:,cline_idx-obs_to_front:cline_idx+obs_end);
    boundary_obs_2 = cline(:,cline_idx+obs_end:cline_idx+obs_to_end)-[(cline_to_left:(cline_half-cline_to_left)/(obs_to_end-obs_end):cline_half);(cline_to_left:(cline_half-cline_to_left)/(obs_to_end-obs_end):cline_half)].*vertical(:,cline_idx+obs_end:cline_idx+obs_to_end);
    boundary_obs_3 = fliplr(cline_right(:,cline_idx+obs_end:cline_idx+obs_to_end));
    boundary_obs_4 = fliplr(right(:,cline_idx-obs_to_front:cline_idx+obs_end));
    obs_right_b = right(:,cline_idx-obs_front:cline_idx+obs_end);
    obs_left_b_1 = fliplr(cline(:,cline_idx-obs_left_front:cline_idx+obs_end) - obs_left*vertical(:,cline_idx-obs_left_front:cline_idx+obs_end));
    obs_left_b_2 = fliplr(right(:,cline_idx-obs_front:cline_idx-obs_left_front)-[(0:(obs_left+cline_to_right)/(obs_front-obs_left_front):(obs_left+cline_to_right));(0:(obs_left+cline_to_right)/(obs_front-obs_left_front):(obs_left+cline_to_right))].*vertical(:,cline_idx-obs_front:cline_idx-obs_left_front));
    seperate = [NaN NaN]';
    boundary_obs_out = [boundary_obs_1 boundary_obs_2 boundary_obs_3 ...
        boundary_obs_4 boundary_obs_1(:,1)];
    boundary_obs_in = [obs_right_b obs_left_b_1 obs_left_b_2];
    boundary_obs = [boundary_obs_out seperate boundary_obs_in];
%     plot(obstacle(1,obs_idx),obstacle(2,obs_idx),'ro');
%     hold on
%     plot(boundary_obs_out(1,:),boundary_obs_out(2,:),'r',boundary_obs_in(1,:),...
%         boundary_obs_in(2,:),'r')
%     hold on
    
    if cline_idx - I_start >= entry_length + entry_to_obs + obs_to_front && I_start + section - cline_idx >= obs_to_end
        boundary_front = [cline_right(:,I_start+entry_length:cline_idx-obs_to_front) fliplr(cline_left(:,I_start+entry_length:cline_idx-obs_to_front)) ...
            cline_right(:,I_start+entry_length)];
        boundary_back = [cline_right(:,cline_idx+obs_to_end:I_start+section) fliplr(cline_left(:,cline_idx+obs_to_end:I_start+section)) ...
            cline_right(:,cline_idx+obs_to_end)];
%         plot(boundary_front(1,:),boundary_front(2,:),'r',boundary_back(1,:),...
%             boundary_back(2,:),'r')
%         hold on
    elseif cline_idx - I_start < entry_length + entry_to_obs + obs_to_front
        boundary_back = [cline_right(:,cline_idx+obs_to_end:I_start+section) fliplr(cline_left(:,cline_idx+obs_to_end:I_start+section)) ...
            cline_right(:,cline_idx+obs_to_end)];
%         plot(boundary_back(1,:),boundary_back(2,:),'r')
%         hold on
    else
        boundary_front = [cline_right(:,I_start+entry_length:cline_idx-obs_to_front) fliplr(cline_left(:,I_start+entry_length:cline_idx-obs_to_front)) ...
            cline_right(:,I_start+entry_length)];
%         plot(boundary_front(1,:),boundary_front(2,:),'r')
%         hold on
    end
    
else
%     plot(boundary(1,:),boundary(2,:),'r');
end

%% search space boundary of entry part
% global boundary_entry
boundary_entry = [];
d_v = (initial(1:2)-cline(:,I_start))'*vertical(:,I_start);
o = [cos(initial(3));sin(initial(3))];
a_d = cross([direction(:,I_start);0],[o;0]);
if norm(direction(:,I_start)-o)<0.01
    a =  0.01;
else
    a = acos(direction(:,I_start)'*o);
end
if cline_idx == 0
    if d_v > 0 && a_d(3) >= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v+entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+[((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+[((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)));((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v <=0 && a_d(3) <= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v-entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+[((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+[((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)));((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v > 0 && a_d(3) < 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + [((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)));((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_left(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    elseif d_v <= 0 && a_d(3) > 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + [((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)));((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_right(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    end
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
elseif cline_idx - I_start < entry_length + entry_to_obs + obs_to_front && cline_idx - I_start > obs_to_front
    if d_v > 0
        boundary_entry = [cline(:,I_start:cline_idx-obs_to_front) + [((d_v+entry_margin):-(d_v-(cline_to_right-entry_margin))/(cline_idx-obs_to_front-I_start):cline_to_right);((d_v+entry_margin):-(d_v-(cline_to_right-entry_margin))/(cline_idx-obs_to_front-I_start):cline_to_right)].*vertical(:,I_start:cline_idx-obs_to_front) ...
            fliplr(left(:,I_start:cline_idx-obs_to_front))];
    else
        boundary_entry = [cline(:,I_start:cline_idx-obs_to_front) + [((d_v-entry_margin):-(d_v+cline_to_left-entry_margin)/(cline_idx-obs_to_front-I_start):-cline_to_left);((d_v-entry_margin):-(d_v+cline_to_left-entry_margin)/(cline_idx-obs_to_front-I_start):-cline_to_left)].*vertical(:,I_start:cline_idx-obs_to_front) ...
            fliplr(right(:,I_start:cline_idx-obs_to_front))];
    end
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
elseif cline_idx - I_start >= entry_length + entry_to_obs + obs_to_front
    if d_v > 0 && a_d(3) >= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v+entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+[((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+[((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)));((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v <=0 && a_d(3) <= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v-entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+[((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+[((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)));((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v > 0 && a_d(3) < 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + [((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)));((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half);((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_left(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    elseif d_v <= 0 && a_d(3) > 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + [((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)));((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)))].*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + [((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half);((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half)].*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_right(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    end
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
end
    
%% find path using heuristic search

path = heuristic(initial,cline(:,I_start+section),boundary,boundary_obs,boundary_front,boundary_back,boundary_entry);
if isempty(path)
    sequence = [];
    return
end
% plot(path(1,:),path(2,:),'b.')

%% compute output sequence
path_need = path(1:3,1:1+ceil(required_length));
sequence = zeros(3,number);
for i = 1:number
    next = 1+i*u*dt;
    next_xy = path_need(1:2,floor(next))+(next-floor(next))*...
        (path_need(1:2,ceil(next))-path_need(1:2,floor(next)));
    v_3 = vecnorm(cline-[next_xy(1)*ones(1,length(cline));next_xy(2)*ones(1,length(cline))]);
    if isempty(v_3)
        lateral = 0;
        c_idx = 0;
    else
        [lateral,c_idx] = min(v_3);
    end
    crossp = cross([direction(:,c_idx);0],[next_xy-cline(:,c_idx);0]);
    if crossp(3)>0
        sequence(:,i)=[next_xy;lateral];
    else
        sequence(:,i)=[next_xy;-lateral];
    end
end
%plot(sequence(1,:),sequence(2,:),'rx')
%figure()
%plot(1:number,sequence(3,:))

% d = path_need(3,floor(next))+(next-floor(next))*...
%         (path_need(3,ceil(next))-path_need(3,floor(next)));

%% shift back point sets if being shifted
% if shift
%     cline = circshift(cline,-shift,2);
%     direction = circshift(direction,-shift,2);
%     vertical = circshift(vertical,-shift,2);
%     right = circshift(right,-shift,2);
%     center = circshift(center,-shift,2);
%     left = circshift(left,-shift,2);
%     cline_right = circshift(cline_right,-shift,2);
%     cline_left = circshift(cline_left,-shift,2);
%     obs_cline = circshift(obs_cline,-shift,2);
% end

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function path = heuristic(start,goal,boundary,boundary_obs,boundary_front,boundary_back,boundary_entry)

% Node vector components:
% x,y,theta,self_id,parent_id,[x1,y1,theta1,...,x4,y4,theta4]
start_node = [start;zeros(15,1)];

% initialize parameters
u = 20;
L = 2.5;
dt = 0.05;
input = [-pi/10,-pi/22,0,pi/22,pi/10];
max_id = 0;
idx_f = 1;
idx_e = 1;
% initialize frontier and explored list
frontier = inf(18,1000);
explored = inf(18,1000);
frontier(:,1) = start_node;

while (min(frontier(1,:)) ~= inf)
    % get current node from frontier
    [~,I] = min(frontier(6,:));
    current_node = frontier(:,I);
    frontier(:,I) = inf(18,1);
    
    % add to explored if not exist
    if min(explored(1,:)) == inf
        %plot(current(1),current(2),'rx')
        %hold on
        max_id = max_id +1;
        current_node(4) = max_id;
        explored(:,idx_e) = current_node;
    elseif min(vecnorm(explored(1:3,:)-[current_node(1)*ones(1,length(explored));current_node(2)*ones(1,length(explored));current_node(3)*ones(1,length(explored))]))>0.005
        %plot(current(1),current(2),'rx')
        %hold on
        max_id = max_id + 1;
        current_node(4) = max_id;
        idx_e = idx_e + 1;
        explored(:,idx_e) = current_node;
    else
        continue
    end
    
    % If found the goal, return the path
    if norm(current_node(1:2)-goal(1:2))<5
        path_all = inf(3,1000);
        current_id = current_node(4);
        idx_p = 1;
        while current_id ~= 1
            v_4 = find(explored(4,:)==current_id);
            if isempty(v_4)
                m = 0;
            else
                m = min(v_4);
            end
            path_append = [explored(1:3,m) fliplr(reshape(explored(7:18,m),3,[]))];
            path_all(:,idx_p:idx_p+4) = path_append;
            idx_p = idx_p+5;
            current_id = explored(5,m);
        end
        path_all(:,idx_p)=explored(1:3,1);
        path = fliplr(path_all(:,1:idx_p));
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
        if collision(x,y,boundary,boundary_obs,boundary_front,boundary_back,boundary_entry)
            %plot(x,y,'bx')
            %hold on
            continue
        else
            child = [x,y,theta,zeros(1,15)]';
            child(6) = norm([x y]'-goal(1:2));
            child(5) = current_node(4);
            child(7:18) = sequence;
            idx_f = idx_f + 1;
            frontier(:,idx_f) = child;
        end
    end
    
end

path = [];
return;

end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function z = collision(x,y,boundary,boundary_obs,boundary_front,boundary_back,boundary_entry)

% global boundary
% global boundary_obs
% global boundary_front
% global boundary_back
% global boundary_entry

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


















