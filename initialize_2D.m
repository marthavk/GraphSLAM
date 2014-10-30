function [poses,map_size,meas_range,mot_noise,meas_noise,velocity,dt,rob_pos,landmarks, num_landmarks, r, x, y] =...
    initialize_2D(map_file, mot_type)

%initialize components
        % robot initial position  

if (~mot_type)
    poses          = 2000;
else
    poses          = 2000;
end
rob_pos        = [14.17;114.5];


map_size           = [600 600];     % size of world
meas_range         = 150;           % range at which we can sense landmarks
mot_noise          = [0.01 0;
                      0 0.01];         % noise in robot motion
meas_noise         = [2 0;
                      0 2];       % noise in the measurements
velocity           = [2.5; 2.5];    % distance by which robot (intends to) move each iteratation
dt                 = 0.1;

%in case of circular motion: radius and center of the circle of the
%movement
r = 200;
x = 200;
y = 200;

%initialization with specific landmarks (map input)    
if (nargin > 0)
    map = load(map_file);   
    landmarks = zeros(2, size(map, 1));
    num_landmarks = size(map,1);
    
    for i=1:num_landmarks
        landmarks(:,i) = map(i, 2:3)';        
    end

%random initialization of 5 landmarks
else    
    num_landmarks = 5;
    landmarks(1,:) = rand(1, num_landmarks)*map_size(1);
    landmarks(2,:) = rand(1, num_landmarks)*map_size(2);
end



end
