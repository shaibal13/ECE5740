%CONSTANT - set trailing on (1) to display the path that the robot
%has taken and off (0) to only show the present position
TRAILING = 0;  %1 is on, 0 is off

B = zeros(1000, 1000);  % setup a 1000 x 1000 pixel board

% Create the walls around the border of the board
B(5, :) = 255;
B(995, :) = 255;
B(:,5) = 255;
B(:, 995) = 255;

% Create an interior wall
% B(570:572, 250:750) = 255;

% Initialize Cat position
RrCat = 400;
RcCat = 400;
RthetaCat = 45;

% Initialize Mouse position
RrMouse = 350;
RcMouse = 350;
RthetaMouse = 35;

% Initialize position buffers (stores previous positions dynamically)
past_positions_cat = []; % Store past (row, col) for Cat
past_positions_mouse = []; % Store past (row, col) for Mouse

% Draw the initial positions
B(RrCat-5:RrCat+5, RcCat-5:RcCat+5) = 254; % CAT
B(RrMouse-5:RrMouse+5, RcMouse-5:RcMouse+5) = 128; % MOUSE

% Load the Fuzzy Inference System
fuzAvoidObstacle = readfis('fuzAvoidObstacle.fis');
fuzSpeed = readfis('fuzSpeed.fis');
fuzCatchMouse = readfis('fuzCatchMouse.fis');
fuzAvoidCat = readfis('fuzAvoidCat.fis');

% Sensors angle (360 degree)
sensor_angles = -180:10:180;
%% Initial speed
speedCat = 4;
speedMouse = 4;
%% Initial distance
distances = zeros(size(sensor_angles));
distances_mouse = zeros(size(sensor_angles));
%% Steps
for step = 1:1500
    % Store past positions
    past_positions_cat = [past_positions_cat; RrCat, RcCat];
    past_positions_mouse = [past_positions_mouse; RrMouse, RcMouse];
    
    % Compute distances for Cat
     %% Get the distance from angle sensor same like part B for cat

    for j = 1:length(sensor_angles)
        distances(j) = GetDist(B, RrCat, RcCat, RthetaCat, sensor_angles(j));
        if (distances(j) == -1 || distances(j) > 100)
             distances(j) = 100;
        end       
    end
    %% Divide the distances for left and right sensor format 

    distLCat = mean(distances(sensor_angles > 0));
    distRCat = mean(distances(sensor_angles < 0));
    %% Get the distangle from other robot

    [distCatToMouse, angleCatToMouse] = getDistAngle(B, RrCat, RcCat, RthetaCat, 128);

    % Compute distances for Mouse
    %% Get the distance from angle sensor same like part B for mouse
    for j = 1:length(sensor_angles)
        distances_mouse(j) = GetDist(B, RrMouse, RcMouse, RthetaMouse, sensor_angles(j));
        if (distances_mouse(j) == -1 || distances_mouse(j) > 100)
             distances_mouse(j) = 100;
        end       
    end
    %% Divide the distances for left and right sensor format 
    distLMouse = mean(distances_mouse(sensor_angles > 0));
    distRMouse = mean(distances_mouse(sensor_angles < 0));
    %% Get the distangle from other robot
    [distMouseToCat, angleMouseToCat] = getDistAngle(B, RrMouse, RcMouse, RthetaMouse, 254);

    %% Compute FIS outputs
    turnCatObstacle = evalfis(fuzAvoidObstacle, [distLCat distRCat]);
    turnCatChase = evalfis(fuzCatchMouse, [distCatToMouse angleCatToMouse]);
    %%Combine the turn angles
    turnCat = 0.1 * turnCatObstacle + 0.9 * turnCatChase;
    %% Get the speed 
    speedCat = evalfis(fuzSpeed, turnCat);
    %% Compute FIS outputs for mouse

    turnMouseObstacle = evalfis(fuzAvoidObstacle, [distLMouse distRMouse]);
    turnMouseEscape = evalfis(fuzAvoidCat, [distMouseToCat angleMouseToCat]);
    %% Get the turn
    turnMouse = 0.9 * turnMouseObstacle + 0.1 * turnMouseEscape;
    %% Get the speed for mouse
    speedMouse = evalfis(fuzSpeed, turnMouse);

    % Move Cat
    [RrCat, RcCat] = MoveRobot(RrCat, RcCat, RthetaCat, speedCat);
    RthetaCat = RthetaCat + turnCat;

    % Move Mouse
    [RrMouse, RcMouse] = MoveRobot(RrMouse, RcMouse, RthetaMouse,speedMouse);
    RthetaMouse = RthetaMouse + turnMouse;
    
    if TRAILING ~= 1 && size(past_positions_cat, 1) > 1
        prev_cat = past_positions_cat(end-1, :);
        prev_mouse = past_positions_mouse(end-1, :);
        B(int64(prev_cat(1))-5:int64(prev_cat(1))+5, int64(prev_cat(2))-5:int64(prev_cat(2))+5) = 0;
        B(int64(prev_mouse(1))-5:int64(prev_mouse(1))+5, int64(prev_mouse(2))-5:int64(prev_mouse(2))+5) = 0;
    end
    %% Draw the robot
    B(int64(RrCat)-5:int64(RrCat)+5, int64(RcCat)-5:int64(RcCat)+5) = 254;
    B(int64(RrMouse)-5:int64(RrMouse)+5, int64(RcMouse)-5:int64(RcMouse)+5) = 128;
    %% CSV generate
    M = [RrCat RcCat RrMouse RcMouse RthetaCat RthetaMouse turnCat turnMouse speedCat speedMouse];
    dlmwrite('partC.csv',M,'-append');
   %% Catch condition
    if ~( (RcCat-5 > RcMouse+5 + 1) || (RcCat+5 < RcMouse-5 - 1) || (RrCat-5 > RrMouse+5 + 1) || (RrCat+5 < RrMouse-5 - 1) )
        disp('Cat catches mouse');
        break;
    end
    %% Draw
    imshow(uint8(B)); drawnow;
end

imshow(uint8(B));