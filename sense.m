function meas = sense(landmarks,rob_pos,meas_range,meas_noise,num_lan)
% Inputs:       landmarks   2xn
%               rob_pos     2x1
%               meas_range  1x1
%               meas_noise  2x2
%               num_lan     1x1 (n)
%
% Output:       meas        nx4

    meas = zeros(num_lan,1);
    count = 1;  
    for i = 1:size(landmarks,2)
        dif = landmarks(:,i) - rob_pos + randn(2,1).*[meas_noise(1,1); meas_noise(2,2)];
        dis = sqrt(dif(1)^2 + dif(2)^2);
        if (dis<meas_range)
            meas(count,1) = i;
            meas(count,2) =  dif(1);
            meas(count,3) =  dif(2);
            meas(count,4) =  dis;
            count = count +1;
        end
        
    end
    meas = meas(1:count - 1,:);
    
end