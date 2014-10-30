function [W,Xi] = update(step,poses,meas,mot_noise,meas_noise,i,W,Xi)
% Inputs:   step       2x1
%           poses      1x1;
%           meas       nx4
%           mot_noise  2x2
%           meas_noise 2x2
%           land_seen  nx1
%           i          1x1
%           W          NxN (N=2*(num_lan+poses)) 
%           Xi         Nx1
%
% Outputs:  W          NxN
%           Xi         Nx1

n = 2*i-1;
%update the motion    
W(n:n+1,n:n+1)              = W(n:n+1,n:n+1)          + eye(2,2)/mot_noise;
W(n:n+1,n+2:n+3)            = W(n:n+1,n+2:n+3)        - eye(2,2)/mot_noise;
W(n+2:n+3,n:n+1)            = W(n+2:n+3,n:n+1)        - eye(2,2)/mot_noise;
W(n+2:n+3,n+2:n+3)          = W(n+2:n+3,n+2:n+3)      + eye(2,2)/mot_noise;

Xi(n:n+1)                   = Xi(n:n+1)               - step./[mot_noise(1,1); mot_noise(2,2)];
Xi(n+2:n+3)                 = Xi(n+2:n+3)             + step./[mot_noise(1,1); mot_noise(2,2)]; 

%update W,Xi with every measurement data    
for k = 1:size(meas,1) 

    m = 2*(meas(k,1) + poses) - 1;
    W(n:n+1, n:n+1)             = W(n:n+1, n:n+1)         + eye(2,2)/meas_noise;
    W(m:m+1, n:n+1)             = W(m:m+1, n:n+1)         - eye(2,2)/meas_noise;
    W(n:n+1, m:m+1)             = W(n:n+1, m:m+1)         - eye(2,2)/meas_noise;
    W(m:m+1, m:m+1)             = W(m:m+1, m:m+1)         + eye(2,2)/meas_noise;

    Xi(n:n+1)                   = Xi(n:n+1)               - (meas(k,2:3)./[meas_noise(1,1) meas_noise(2,2)])';
    Xi(m:m+1)                   = Xi(m:m+1)               + (meas(k,2:3)./[meas_noise(1,1) meas_noise(2,2)])';

end


end
