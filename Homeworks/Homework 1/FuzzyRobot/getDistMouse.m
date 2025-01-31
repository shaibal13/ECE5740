function [dist] = getDistMouse(B, Rr, Rc, Rtheta, SensorThetaOffset)

    %compute the distance to an obstacle
    %an obstacle is a 255 value on the board
    %start with dist = -1, i.e. no obstacle found
    dist = -1;

    %move the distance center beam along a path that is
    %the robot heading plus the sensor angle offset
    c = cosd(Rtheta + SensorThetaOffset);
    r = sind(Rtheta + SensorThetaOffset);
    
    %evaluate 100 steps (i.e. the sensor can go 100 units out from
    %the robot)
    for i=1:100
        Rc2 = Rc + c * i;
        Rr2 = Rr - r * i;
        
        %if we haven't gone off the board yet
        if ((int64(Rc2) >= 1) && (int64(Rr2) >= 1) && (int64(Rc2) <= 1000) && (int64(Rr2) <= 1000))
            B(int64(Rr2), int64(Rc2))
            %if there is an obstacle, value 255, and we are not in 
            %the first 6 units of movement since that is too close 
            %to the robot, then we found an obstacle
            if ((B(int64(Rr2), int64(Rc2)) == 128))
                %compute the distance to the obstacle found
                dist = sqrt( (Rr-Rr2)^2 + (Rc-Rc2)^2);
           
                %since we found an obstacle, no need to calculate 
                %any further, return.
                return;
            end
        else
            %we are off the board with the sensor, return
            return;
        end
    end

end