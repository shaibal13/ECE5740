function [Rr, Rc] = MoveRobot(Rr, Rc, Rtheta, Speed)
% This function returns Rr and Rc - the new row and column
% these will be non-integer numbers (containing decimals)

    %calculate the new column and row using trig on the unit circle
    %the robot is pointed in direction Rtheta and needs to move Speed
    %units in that direction
    c = cosd(Rtheta);
    r = sind(Rtheta);
    
    %scale the trig by the speed and add to column (moving to the right)
    %and subtract from row (moving upwards).  The add / subtract
    %is because trig functions will be positive up and to the right.
    Rc = Rc + c * Speed;
    Rr = Rr - r * Speed;

end