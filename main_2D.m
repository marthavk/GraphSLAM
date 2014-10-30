%load initiale state
[poses,map_size,meas_range,mot_noise,meas_noise,velocity,dt,rob_pos,landmarks, num_lan] =...
    initialize_2D('landmarks50.txt');

%initialize Wmega, Xi
N           = 2*(num_lan+poses);
W           = zeros(N,N);
Xi          = zeros(N,1);
meas        = zeros(num_lan,4);  %measurement matrix

%initial position of robot
W(1,1) = 1000000;
W(2,2) = 1000000;
rob_posm = [];

%variables for illustration
ROB_POS(:,1)=rob_pos;


Xi(1:2)  = [rob_pos(1)*1000000; rob_pos(2)*1000000];

step = dt*velocity;

for i = 1:poses-1            
    meas = sense(landmarks,rob_pos,meas_range,meas_noise,num_lan);
    MEAS(:,i)=meas(2:3)';
%     [rob_pos,step] = move(rob_pos,dt,velocity,mot_noise);  
    [rob_pos,step] = move_circular(rob_pos, mot_noise, 120, 320, 240); 
    
    ROB_POS(:,i+1) = rob_pos;       
    
    [W,Xi] = update(step,poses,meas,mot_noise,meas_noise,i,W,Xi);
    rob_posm = [rob_posm rob_pos];
end

[W, Xi] = deleteZeros(W,Xi,N,poses);
[pos_landm,pos]=solve(W,Xi,poses);

% ROB_POS
figure
hold on
plot (landmarks(1,:), landmarks(2,:), 'o');
plot(pos_landm(:,1), pos_landm(:,2), 'rx');
plot(ROB_POS(1,:), ROB_POS(2,:), 'b');
plot(pos(:,1), pos(:,2), 'r.');
