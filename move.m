function [rob_pos,step]=move(rob_pos,dt,velocity,mot_noise)

% Inputs:   rob_pos   2x1
%           dt        1x1
%           velocity  2x1
%           mot_noise 2x2
%
% Outputs: rob_pos    2x1
%          step       2x1

step = dt.*velocity;
% rob_pos = rob_pos + ones(2,1)*dt.*velocity;
% step = rob_pos - step;
rob_pos = rob_pos + step + randn(2,1).*[mot_noise(1,1); mot_noise(2,2)] ;

end