clc
clear
close all
%set up the potential field for the map
map=ones(500);
map(300:330,300:330)=zeros(31);
%set the road
map(50,1:450)=zeros(1,450);
map(350,1:450)=zeros(1,450);
map(240,1:450)=ones(1,450)*0.9;
map=int16(map);
%%%%%%Set up parameters%%%%%%
source=[300 10];%the initial position in Y X fromate
goal=[300 450]; %the goal position in Y X formate
robotDirection=0; % initial heading direction
robotSize=[2 2]; %length and breadth
robotSpeed=20; % arbitrary units 
maxRobotSpeed=10; % arbitrary units 
S=2; % safety distance
distanceThreshold=10; % a threshold distace. points within this threshold can be taken as same. 
maxAcceleration=6; % maximum speed change per unit time
maxTurn=30*pi/180; % potential outputs to turn are restriect to -60 and 60 degrees.
k=3.5; % degree of calculating potential
attractivePotentialScaling=3000000; % scaling factor for attractive potential
repulsivePotentialScaling=300000; % scaling factor for repulsive potential
minAttractivePotential=1; % minimum attractive potential at any point
%%%%%%%start planning%%%%%%%%

currentPosition=source;
currentDirection=robotDirection;
robotDiagnal=sqrt((robotSize(1)/2)^2+(robotSize(2)/2)^2);
pathFound=false;
pathCost=0;
t=1;
imshow(map==1);
rectangle('position',[1 1 size(map)-1],'edgecolor','k')
pathlength=0;

%check the initial position and target position is avaliable
if ~plotRobot(currentPosition,currentDirection,map,robotDiagnal)
   error('start position lies on an obstacle or outside map');
end
if ~feasurePoint(goal,map)
   error('Target position lies on an obstacle or outside map');
end
tic
%find the path
while ~pathFound
    %calculate the distance from obstacel at front
    distance=robotSize(1)/2+1;
    while true
        x=currentPosition+distance*[sin(currentDirection) cos(currentDirection)];
        x=int16(x);
        if ~feasurePoint(x,map)
            break;
        end
        distance=distance+1;
        
    end
    disFront=distance-robotSize(1)/2; %find the fron distance eliminate robot size
    
      %calculate the distance from obstacel at left
    distance=robotSize(1)/2+1;
    while true
        x=currentPosition+distance*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)];
         x=int16(x);
        if ~feasurePoint(x,map)
            break;
        end
        distance=distance+1;
        
    end
    disLeft=distance-robotSize(1)/2; %find the fron distance eliminate robot size
    
    %calculate the distance from obstacel at right
    distance=robotSize(1)/2+1;
    while true
        x=currentPosition+distance*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)];
         x=int16(x);
        if ~feasurePoint(x,map)
            break;
        end
        distance=distance+1;
        
    end
    disRight=distance-robotSize(1)/2; %find the fron distance eliminate robot size
    
    %calculate the distance from obstacel at left diagnoal
    distance=robotDiagnal+1;
    while true
        x=currentPosition+distance*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)];
         x=int16(x);
        if ~feasurePoint(x,map)
            break;
        end
        distance=distance+1;
        
    end
    disLeftDiagonal=distance-robotDiagnal; %find the fron distance eliminate robot size
    
    %calculate the distance from obstacel at right diagnoal
    distance=robotDiagnal+1;
    while true
        x=currentPosition+distance*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)];
         x=int16(x);
        if ~feasurePoint(x,map)
            break;
        end
        distance=distance+1;
        
    end
    disRightDiagonal=distance-robotDiagnal; %find the fron distance eliminate robot size
    
    %calculate the angle to the goal
    angleGoal=atan2(goal(1)-currentPosition(1),goal(2)-currentPosition(2));
    %calculate the distance to the goal
    distanceGoal=sqrt((goal(1)-currentPosition(1))^2+(goal(2)-currentPosition(2))^2);
    %judege whether found the goal
    if distanceGoal<distanceThreshold
        pathFound=true;
    end
    
    %compute potentails
    repulsivePotential=(1.0/disFront)^k*[sin(currentDirection) cos(currentDirection)] + ...
     (1.0/disLeft)^k*[sin(currentDirection-pi/2) cos(currentDirection-pi/2)] + ...
     (1.0/disRight)^k*[sin(currentDirection+pi/2) cos(currentDirection+pi/2)] + ...
     (1.0/disLeftDiagonal)^k*[sin(currentDirection-pi/4) cos(currentDirection-pi/4)] + ...
     (1.0/disRightDiagonal)^k*[sin(currentDirection+pi/4) cos(currentDirection+pi/4)];
   
    attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling ...
        minAttractivePotential])*[sin(angleGoal) cos(angleGoal)];
    totalPotential=attractivePotential-repulsivePotential*repulsivePotentialScaling;
    
    %perform steering
     preferredSteer=atan2(robotSpeed*sin(currentDirection)+totalPotential(1), ...
         robotSpeed*cos(currentDirection)+totalPotential(2))-currentDirection;
     %limit the steering angle in pi to -pi
     preferredSteer=minimizeAngle(preferredSteer);
     
     %limit the steering angle less than the maximam steering angle
     preferredSteer=max(-maxTurn,preferredSteer);
     preferredSteer=min(maxTurn,preferredSteer);
      
     currentDirection=currentDirection+preferredSteer;
     
     % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
     preferredSpeed=sqrt(sum((robotSpeed*[sin(currentDirection) cos(currentDirection)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     if robotSpeed==0 
         error('robot had to stop to avoid collission'); 
     end
     
     %update the new position 
     newPosition=currentPosition+robotSpeed*[sin(currentDirection),cos(currentDirection)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     
     if ~feasurePoint(int16(currentPosition),map)
         error('collission recorded'); 
     end
      % plotting robot
     if ~plotRobot(currentPosition,currentDirection,map,robotDiagnal)
        error('collission recorded');
     end
end

fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathCost); 


function test=plotRobot(position,direction ,map,step)

%plot the robot trajectory according to current position and direction
corner1=position+step*[sin(direction-pi/4), cos(direction-pi/4)];
corner2=position+step*[sin(direction+pi/4), cos(direction+pi/4)]; 
corner3=position+step*[sin(direction+pi-pi/4), cos(direction+pi-pi/4)];
corner4=position+step*[sin(direction+pi+pi/4), cos(direction+pi+pi/4)];
line([corner1(2);corner2(2);corner3(2);corner4(2);corner1(2)],...
 [corner1(1);corner2(1);corner3(1);corner4(1);corner1(1)],'color','blue','LineWidth',2);
if (~feasurePoint(int16(corner1),map) || ~feasurePoint(int16(corner2),map)...
        || ~feasurePoint(int16(corner3),map) || ~feasurePoint(int16(corner4),map))
    test=false;
else 
    test=true;
end
end


function test=feasurePoint(point,map)

test=true;
%check whether the point is in the boundary and no collision
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 ...
    && point(2)<=size(map,2) && map(point(1),point(2))==1)
    test=false;
end


end

function angle_new=minimizeAngle(angle)
%convet angle to -pi to pi
 while angle>pi
     angle=angle-2*pi;
 end
 while angle<-pi
     angle=angle+2*pi;
 end

angle_new=angle;
end

function h=distanceCost(a,b)
h = sqrt(sum(a-b).^2);
end