function [rob_pos,step]=move_circular(rob_pos, mot_noise, r, x, y)

% Inputs:   rob_pos   2x1
%           dt        1x1
%           velocity  2x1
%           mot_noise 2x2
%
% Outputs: rob_pos    2x1
%          step       2x1

%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)

%moves about one degree in each step
ang_step = -0.2*0.017 ;
ang_in_rad = atan2(rob_pos(2)-y, rob_pos(1)-x);
ang = ang_in_rad + ang_step;
xp=r*cos(ang);
yp=r*sin(ang);
step = [rob_pos];
rob_pos = [x+xp; y+yp];
step = rob_pos - step;
rob_pos = rob_pos  + randn(2,1).*[mot_noise(1,1); mot_noise(2,2)];
end



