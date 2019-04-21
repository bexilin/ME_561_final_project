close all
clear
load('required_points.mat')
cline_to_left = 9;
cline_to_right = 3;

%% parameters you need to specify
% definition of parameters is demonstrated in prediction.m file
u = 10;
dt = 0.1;
number = 40;

%% test: the start point lateral position vary within road width, the heading angle vary from -pi/3 to pi/3
% % *note: (1) path planning fails in some cases (currently found):
% %            i). initial position is too close to obstacle (this situation is actually unsolvable unless allowing reverse motion)
% %            ii). road point sets are very dense in the planning section, affecting the shape and size of search space severely       
% %        (2) turn on plots in prediction for showing results
idx = randi(size(cline,2));
p = cline(:,idx) + (12*rand-cline_to_left)*vertical(:,idx);
d = atan(direction(2,idx)/direction(1,idx));
if direction(1,idx)<0
    d = d + pi;
end
d = d + 2*pi/3*rand-pi/3;
initial = [p;d];
tic
sequence = prediction(initial,u,dt,number);
toc
