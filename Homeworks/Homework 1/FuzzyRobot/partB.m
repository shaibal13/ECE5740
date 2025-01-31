
%CONSTANT - set trailing on (1) to display the path that the robot
%has taken and off (0) to only show the present position
TRAILING = 0;  %1 is one, 0 is off

B = zeros(1000, 1000);  % setup a 1000 x 1000 pixel board

%create the walls around the border of the board
%walls will be 255, the max of 8-bit grayscale which is white
%we will create the walls to be 5 pixels from the actual edge of
%the board so that the robot (which is more than a point pixel) can be 
%drawn outside the wall with valid matrix indices
B(5, :) = 255;
B(995, :) = 255;
B(:,5) = 255;
B(:, 995)= 255;

%create 3 obstacle interior wall
B(570:572, 250:750) = 255;
B(70: 250, 570:572) = 255;
B(250:252, 250:750) = 255;

%set the robot starting position.  We will use variables Rr and Rc to 
%track and set the current position of the robot (Rr, Rc)
Rr = 700;
Rc = 700;
Rtheta = 145;    %the direction the robot is pointing in
                 %using 0 degrees to the right (unit circle)

% draw the robot as a 7-by-7 square instead of a pixel point
B(Rr-3:Rr+3, Rc-3:Rc+3) = 254;

% load the Fuzzy Inference System
fuzAvoidObstacle = readfis('fuzAvoidObstacle.fis');
% New Fuzzy inference system
fuzSpeed = readfis('fuzSpeed.fis');

% Sensors angle
sensor_angles = [-40 -30 -20 -10 0 10 20 30 40];
%sensor_angles = 10:10:40;
distances = zeros(size(sensor_angles));
speed=2;
%for 1500 steps, move the robot
for i = 1:1500
    
    if (TRAILING ~= 1)
        %if we are not tracking the trail, black out the robot
        
        B(Rr-3:Rr+3, Rc-3:Rc+3) = 0;  %black out robot position
            %before it moves it
    end
        
    %move the robot at (Rr, Rc) in direction Rtheta, 2 units
    %2 is basically the speed of motion
    [Rr, Rc] = MoveRobot(Rr, Rc, Rtheta, speed);
    
    %Rr and Rc are numbers with decimals
    %to move the robot on the Board, we need to convert those
    %to row and column integers
    Rri = int64(Rr);
    Rci = int64(Rc);    
    %the line below will turn the robot 3 degrees every iteration
    %this is how we got it to go in a circle while we were developing
    %the simulator
    %Rtheta = Rtheta + 3;  
    %draw the robot in its new position 
    B(Rri-3:Rri+3, Rci-3:Rci+3) = 254;
    for j = 1:length(sensor_angles)
        distances(j) = GetDist(B, Rr, Rc, Rtheta,sensor_angles(j) );
        %if the distance is -1 or > 100, then saturate those to 100
         %which means low/no signal
        if (distances(j) == -1 || distances(j) > 100)
             distances(j) = 100;
        end       
    end
    
    % Aggregate sensor data:
    % Take the minimum distance excluding 0° forward 
    distL = mean(distances(6:end));  % Angles: 10, 20, 30, 40 -Left one
    distR = mean(distances(1:4));  % Angles: -40, -30, -20, -10 -Right one  
    %using fuzzyLogicDesigner giving the distL and distR sensor
    %readings as input to get a turn angle.
    turn = evalfis(fuzAvoidObstacle, [distL distR]);
    % This fuzSpeed fuzzy logic designer will input the turn from previous
    % fuzzy logic designer and output the speed with will then pass to
    % MoveRobot function to get the new position.
    speed = evalfis(fuzSpeed, turn);
    
    % [Rr, Rc] = MoveRobot(Rr, Rc, Rtheta, speed);
    %apply the turn angle by updated Rtheta.

    Rtheta = Rtheta + turn;
    
    %display variables on the console for reference
    disp(Rr);
    disp(Rc);
    disp(distL);
    disp(distR);
    disp(Rtheta);
    disp(turn);
    %a much better way to handle logging data is using a comma
    %separated value (CSV) file which will open in Excel instead
    %of displaying it on the Command Window
    M = [Rr Rc distL distR Rtheta turn];
    dlmwrite('partB.csv',M,'-append');

    %draw the current image now.  This will make the robot motion
    %appear like a video, however, it will slow down this algorithm
    %since it is displaying it on the screen every time it moves the robot
    imshow(uint8(B));
    drawnow;
end


imshow(uint8(B));




