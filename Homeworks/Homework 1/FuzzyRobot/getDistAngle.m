function [distance, angle] = getDistAngle(B, Rr, Rc, Rtheta, otherRobotPixel)
    angles = -180:10:180;
    maxRange = 100; 
    distance = 100;  
    angle = 0;

    for a = angles
        current_angle = Rtheta + a;  
        d = -1; % Track distance only if target is found
        step = 1;
        c = cosd(current_angle);
        r = sind(current_angle);

        for i = 1:step:maxRange
            row = Rr - r*i;
            col = Rc + c*i;

            if row<1 || row>size(B,1) || col<1 || col>size(B,2)
                break;
            end

            if i <= 6
                continue;
            end

            % If a wall is hit, stop this scan
            if B(int64(row), int64(col)) == 255 
                break;
            % If the target is found, track it
            elseif B(int64(row), int64(col)) == otherRobotPixel
                d = sqrt((Rr - row)^2 + (Rc - col)^2);
                % Stop scanning in this direction
                break;
            end
        end
          % Only update if a valid distance was found
        if d > 0 && d < distance
            distance = d;
            angle = a;  
        end
    end
end
