function run_GraphSLAM( mapfile, motion_type, verbose )
%verbose = 0: only graphical information given
%verbose = 1: + information about errors in position and landmark estimation
%verbose = 2: + information about estimated position and landmarks
%motion_type = 0: forward motion along a diagonal line
%motion_type = 1: circular motion

if (nargin < 3)
    verbose = 0;
end

%load initiale state
[poses,map_size,meas_range,mot_noise,meas_noise,velocity,dt,rob_pos,landmarks, num_lan, r, x, y] =...
    initialize_2D(mapfile, motion_type);

%initialize Wmega, Xi
N           = 2*(num_lan+poses);
W           = zeros(N,N);
Xi          = zeros(N,1);
meas        = zeros(num_lan,4);  %measurement matrix

%initial position of robot
W(1,1) = 100000;
W(2,2) = 100000;
rob_posm = [];

Xi(1:2)  = [rob_pos(1)*100000; rob_pos(2)*100000];

ROB_POS = rob_pos;

for i = 1:poses-1            
    meas = sense(landmarks,rob_pos,meas_range,meas_noise,num_lan);
    
    
    if (~motion_type)
        [rob_pos,step] = move(rob_pos,dt,velocity,mot_noise);    
    else
        [rob_pos,step] = move_circular(rob_pos, mot_noise, r, x, y); 
    end
    ROB_POS(:,i+1) = rob_pos; 
          
    
    [W,Xi] = update(step,poses,meas,mot_noise,meas_noise,i,W,Xi);
    rob_posm = [rob_posm rob_pos];
end

% for i = 1:poses/2
%     meas = sense(landmarks,rob_pos,meas_range,meas_noise,num_lan);
%     [rob_pos,step] = move(rob_pos,dt,velocity,mot_noise);  
%     [W,Xi] = update(step,poses,meas,mot_noise,meas_noise,i,W,Xi);
%     rob_posm = [rob_posm rob_pos];
% end
% 
% for i=(poses/2+1):(poses-1)
%      meas = sense(landmarks,rob_pos,meas_range,meas_noise,num_lan);
%     [rob_pos,step] = move_circular(rob_pos, mot_noise, 120, 320, 240); 
%     [W,Xi] = update(step,poses,meas,mot_noise,meas_noise,i,W,Xi);
%      rob_posm = [rob_posm rob_pos];     
% end

[W, Xi, landmarks] = deleteZeros(W,Xi,N,poses,landmarks);
[pos_landm,pos]=solve(W,Xi,poses);
if (verbose>1)  
    disp(sprintf('estimated robot positions:'));
    disp(pos);
    disp(sprintf('estimated landmark positions:'));
    disp(pos_landm);
end


figure
hold on
plot (landmarks(1,:), landmarks(2,:), 'o');
plot(pos_landm(:,1), pos_landm(:,2), 'rx');
plot(ROB_POS(1,:), ROB_POS(2,:), 'b');
plot(pos(:,1), pos(:,2), 'r.');

%Errors:
ROB_POS = ROB_POS';
landmarks = landmarks';
E_POS = sqrt((pos(:,1)-ROB_POS(:,1)).^2 + (pos(:,2)-ROB_POS(:,2)).^2);
E_LANDM = sqrt((pos_landm(:,1)-landmarks(:,1)).^2 + (pos_landm(:,2) - landmarks(:,2)).^2);
std(E_POS);
std(E_LANDM);
if (verbose >0)
    disp(sprintf('Standard deviation of error in position: %f pixels ', std(E_POS)));
    disp(sprintf('Standard deviation of error in landmark localization: %f pixels ', std(E_LANDM)));
end

