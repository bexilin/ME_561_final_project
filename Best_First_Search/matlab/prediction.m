function sequence = prediction(initial,u,dt,number)

% % inputs: initial -> unit:[m,m,radian]' size:3X1, initial position and heading angle of vehicle
% %         u -> unit:(m/s), constant longitudinal velocity
% %         dt -> unit:s, time interval between neighbouring points of planned trajectory
% %         number -> quantity of points needed in planned trajectory
% % output: sequence -> unit:[m,m,m]'(for each column) size:3Xnumber, each column reflects the 
% %         position and lateral error from the center line of right lane
% % *note: (1) plots can be turned on to show the road section, search space
% %        and planned trajectory
% %        (2) the function is designed to take in at most one obstalce

global cline obstacle direction vertical right center left cline_right cline_left obs_cline 

%% required length of the planned trajectory
required_length = u*dt*number;

%% parameters for specifying search space boundaries
cline_to_right = 3;
cline_to_left = 9;
cline_half = 0.65;
section = 300;
entry_boundary = 40;
boundary_scale = 0.2;
entry_margin = 1;
entry_length = 140;
entry_to_obs = 20;
obs_to_front = 100;
obs_to_end = 70;
obs_front = 80;
obs_end = 10;
obs_left = 4;
obs_left_front = 40;

%% compute the section of road for prediction
[~,I_start] = min(vecnorm(cline-initial(1:2)));

% % shift point sets if index is out of range (when planning starts near the  
% % end of road point sets)
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

cline_part = cline(:,I_start:I_start+section);

% % check if the length of section is longer than required length, if not,
% % increase it to length equals to or greater than 1.2 times required length
section_length = sum(vecnorm(diff(cline_part,1,2)));
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
        
    cline_part = cline(:,I_start:I_start+section);
end

right_part = right(:,I_start:I_start+section);
center_part = center(:,I_start:I_start+section);
left_part = left(:,I_start:I_start+section);
cline_right_part = cline_right(:,I_start:I_start+section);
cline_left_part = cline_left(:,I_start:I_start+section);

%% boundaries of lane-keeping section when no obstacle exists in the road section 
global boundary
boundary = [cline_right_part(:,1+entry_length:end) fliplr(cline_left_part(:,1+entry_length:end)) cline_right_part(:,1+entry_length)];

% % plots showing the boundaries and center line of the road section, as 
% % well as the lane-keeping line (center line of right lane)

% plot(cline_part(1,:),cline_part(2,:),'g',right_part(1,:),right_part(2,:),...
%     'k',left_part(1,:),left_part(2,:),'k',center_part(1,:),center_part(2,:),'k--')
% hold on

%% boundaries of obstalce avoidance section when an obstacle exists in the road section
global boundary_obs
global boundary_front
global boundary_back
boundary_obs = [];
boundary_front = [];
boundary_back = [];
cline_idx =[];
obs_idx = find(ismember(obs_cline,I_start:I_start+section));
if ~isempty(obs_idx)
    cline_idx = obs_cline(obs_idx);
    boundary_obs_1 = left(:,cline_idx-obs_to_front:cline_idx+obs_end);
    boundary_obs_2 = cline(:,cline_idx+obs_end:cline_idx+obs_to_end)-(cline_to_left:(cline_half-cline_to_left)/(obs_to_end-obs_end):cline_half).*vertical(:,cline_idx+obs_end:cline_idx+obs_to_end);
    boundary_obs_3 = fliplr(cline_right(:,cline_idx+obs_end:cline_idx+obs_to_end));
    boundary_obs_4 = fliplr(right(:,cline_idx-obs_to_front:cline_idx+obs_end));
    obs_right_b = right(:,cline_idx-obs_front:cline_idx+obs_end);
    obs_left_b_1 = fliplr(cline(:,cline_idx-obs_left_front:cline_idx+obs_end) - obs_left*vertical(:,cline_idx-obs_left_front:cline_idx+obs_end));
    obs_left_b_2 = fliplr(right(:,cline_idx-obs_front:cline_idx-obs_left_front)-(0:(obs_left+cline_to_right)/(obs_front-obs_left_front):(obs_left+cline_to_right)).*vertical(:,cline_idx-obs_front:cline_idx-obs_left_front));
    seperate = [NaN NaN]';
    boundary_obs_out = [boundary_obs_1 boundary_obs_2 boundary_obs_3 ...
        boundary_obs_4 boundary_obs_1(:,1)];
    boundary_obs_in = [obs_right_b obs_left_b_1 obs_left_b_2];
    boundary_obs = [boundary_obs_out seperate boundary_obs_in];
    
% % plots showing the obstacle position and boundaries of obstalce avoidance section
    
%     plot(obstacle(1,obs_idx),obstacle(2,obs_idx),'ro');
%     hold on
%     plot(boundary_obs_out(1,:),boundary_obs_out(2,:),'r',boundary_obs_in(1,:),...
%         boundary_obs_in(2,:),'r')
%     hold on
 
%% boundaries of lane-keeping section when an obstacle exists in the road section  
    if cline_idx - I_start >= entry_length + entry_to_obs + obs_to_front && I_start + section - cline_idx >= obs_to_end
        boundary_front = [cline_right(:,I_start+entry_length:cline_idx-obs_to_front) fliplr(cline_left(:,I_start+entry_length:cline_idx-obs_to_front)) ...
            cline_right(:,I_start+entry_length)];
        boundary_back = [cline_right(:,cline_idx+obs_to_end:I_start+section) fliplr(cline_left(:,cline_idx+obs_to_end:I_start+section)) ...
            cline_right(:,cline_idx+obs_to_end)];

% % plots showing boundaries of lane-keeping section
        
%         plot(boundary_front(1,:),boundary_front(2,:),'r',boundary_back(1,:),...
%             boundary_back(2,:),'r')
%         hold on
    elseif cline_idx - I_start < entry_length + entry_to_obs + obs_to_front
        boundary_back = [cline_right(:,cline_idx+obs_to_end:I_start+section) fliplr(cline_left(:,cline_idx+obs_to_end:I_start+section)) ...
            cline_right(:,cline_idx+obs_to_end)];
        
% % plots showing boundaries of lane-keeping section
        
%         plot(boundary_back(1,:),boundary_back(2,:),'r')
%         hold on
    else
        boundary_front = [cline_right(:,I_start+entry_length:cline_idx-obs_to_front) fliplr(cline_left(:,I_start+entry_length:cline_idx-obs_to_front)) ...
            cline_right(:,I_start+entry_length)];
        
% % plots showing boundaries of lane-keeping section
        
%         plot(boundary_front(1,:),boundary_front(2,:),'r')
%         hold on
    end
    
else
    
% % plots showing boundaries of lane-keeping section
    
%     plot(boundary(1,:),boundary(2,:),'r');
end

%% boundaries of entry section
global boundary_entry
boundary_entry = [];
d_v = (initial(1:2)-cline_part(:,1))'*vertical(:,I_start);
o = [cos(initial(3));sin(initial(3))];
a_d = cross([direction(:,I_start);0],[o;0]);
if norm(direction(:,I_start)-o)<0.01
    a =  0.01;
else
    a = acos(direction(:,I_start)'*o);
end
if isempty(cline_idx)
    if d_v > 0 && a_d(3) >= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v+entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v <=0 && a_d(3) <= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v-entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v > 0 && a_d(3) < 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + ((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_left(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    elseif d_v <= 0 && a_d(3) > 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + ((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_right(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    end
    
% % plots showing boundaries of entry section
    
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
elseif cline_idx - I_start < entry_length + entry_to_obs + obs_to_front && cline_idx - I_start > obs_to_front
    if d_v > 0
        boundary_entry = [cline(:,I_start:cline_idx-obs_to_front) + ((d_v+entry_margin):-(d_v-(cline_to_right-entry_margin))/(cline_idx-obs_to_front-I_start):cline_to_right).*vertical(:,I_start:cline_idx-obs_to_front) ...
            fliplr(left(:,I_start:cline_idx-obs_to_front))];
    else
        boundary_entry = [cline(:,I_start:cline_idx-obs_to_front) + ((d_v-entry_margin):-(d_v+cline_to_left-entry_margin)/(cline_idx-obs_to_front-I_start):-cline_to_left).*vertical(:,I_start:cline_idx-obs_to_front) ...
            fliplr(right(:,I_start:cline_idx-obs_to_front))];
    end
    
% % plots showing boundaries of entry section
    
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
elseif cline_idx - I_start >= entry_length + entry_to_obs + obs_to_front
    if d_v > 0 && a_d(3) >= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v+entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v+entry_margin):-(d_v+entry_margin - cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v <=0 && a_d(3) <= 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + (d_v-entry_margin)*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v-entry_margin):-(d_v-(entry_margin - cline_half))/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline(:,I_start+entry_boundary:I_start+entry_length)+((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length));
        boundary_entry_4 = fliplr(cline(:,I_start:I_start+entry_boundary)+((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 boundary_entry_4 ...
            boundary_entry_1(:,1)];
    elseif d_v > 0 && a_d(3) < 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + ((d_v+entry_margin):boundary_scale*tan(a):(d_v+entry_margin+boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v+entry_margin+boundary_scale*entry_boundary*tan(a)):-(d_v+entry_margin+boundary_scale*entry_boundary*tan(a)-cline_half)/(entry_length-entry_boundary):cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_left(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    elseif d_v <= 0 && a_d(3) > 0
        boundary_entry_1 = cline(:,I_start:I_start+entry_boundary) + ((d_v-entry_margin):-boundary_scale*tan(a):(d_v-entry_margin-boundary_scale*entry_boundary*tan(a))).*vertical(:,I_start:I_start+entry_boundary);
        boundary_entry_2 = cline(:,I_start+entry_boundary:I_start+entry_length) + ((d_v-entry_margin-boundary_scale*entry_boundary*tan(a)):-(d_v-entry_margin-boundary_scale*entry_boundary*tan(a)+cline_half)/(entry_length-entry_boundary):-cline_half).*vertical(:,I_start+entry_boundary:I_start+entry_length);
        boundary_entry_3 = fliplr(cline_right(:,I_start:I_start+entry_length));
        boundary_entry = [boundary_entry_1 boundary_entry_2 boundary_entry_3 ...
            boundary_entry_1(:,1)];
    end
    
% % plots showing boundaries of entry section
    
%     plot(boundary_entry(1,:),boundary_entry(2,:),'r');
end
    
%% find trajectory path using best first search, return empty sequence if not found
path = heuristic(initial,cline_part(:,end));
if isempty(path)
    'path planning failure'
    sequence = [];
    return
end

%% compute output trajectory sequence
path_need = path(1:3,1:1+ceil(required_length));
sequence = zeros(3,number);
for i = 1:number
    next = 1+i*u*dt;
    next_xy = path_need(1:2,floor(next))+(next-floor(next))*...
        (path_need(1:2,ceil(next))-path_need(1:2,floor(next)));
    [lateral,c_idx] = min(vecnorm(cline-next_xy));
    crossp = cross([direction(:,c_idx);0],[next_xy-cline(:,c_idx);0]);
    if crossp(3)>0
        sequence(:,i)=[next_xy;lateral];
    else
        sequence(:,i)=[next_xy;-lateral];
    end
end

% % plots positions and lateral errors of trajectory sequence 

%plot(sequence(1,:),sequence(2,:),'bx')
%figure()
%plot(1:number,sequence(3,:))


%% shift back point sets if being shifted
if shift
    cline = circshift(cline,-shift,2);
    direction = circshift(direction,-shift,2);
    vertical = circshift(vertical,-shift,2);
    right = circshift(right,-shift,2);
    center = circshift(center,-shift,2);
    left = circshift(left,-shift,2);
    cline_right = circshift(cline_right,-shift,2);
    cline_left = circshift(cline_left,-shift,2);
    obs_cline = circshift(obs_cline,-shift,2);
end
