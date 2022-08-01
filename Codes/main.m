clc;
clear;

%-------------------Initilization (Variable)------------   
%Only feedforward 
%K_p=zeros(6);
%K_i=zeros(6);

%feedforward+proportional 
%K_p=eye(6);
%K_i=zeros(6);

%Best case
K_p = 6*eye(6);
K_i = 4*eye(6);

%Overshoot
%K_p=30*eye(6);
%K_i=4*eye(6);



%current_state = [1.57,-0.2,0, 0, 0, 0.2, -1.3, 0, -pi/4, pi/4, -pi/4, pi/4];
current_state = [0,-0.2,0, 0, 0, 0.2, -1.3, 0, -pi/4, pi/4, -pi/4, pi/4];
%thetalist = [-pi/4;0;-pi/4;0;0];
thetalist = [0;0;0;0;0];
%-------------------------------------------------------

%---------------------Initilization (Constants)--------------------

del_t = 0.01;
T_b0 = [[1, 0, 0, 0.1662]; [0, 1, 0, 0]; [0, 0, 1, 0.0026]; [0, 0, 0, 1]];
T_sb = [[cos(current_state(1)), -sin(current_state(1)), 0, current_state(2)]; 
         [sin(current_state(1)), cos(current_state(1)), 0, current_state(3)]; 
         [0, 0, 1, 0.0963]; 
         [0, 0, 0, 1]];
T_s0 = T_sb*T_b0;


Blist = [[0; 0; 1;   0; 0.033; 0], ...
       [0; -1; 0;   -0.5076;   0;   0], ...
       [0; -1; 0;   -0.3526;   0;   0], ...
       [0; -1; 0; -0.2176; 0; 0], ...
       [0; 0; 1; 0; 0; 0]];  

M_0e = [[1, 0, 0, 0.033]; [0, 1, 0, 0]; [0, 0, 1, 0.6546]; [0, 0, 0, 1]];

max_v = 10; % Maximum Joint Speeds

%----------------------Main Loop -------------------------
%Desired Trajectory generation 
traj=TrajectoryGenerator();

current_state_append = [];
X_err_append = [];
velocities = zeros(1,9);

T_0e = FKinBody(M_0e,Blist,thetalist);
X = T_s0*T_0e;


for i = 1:4499
    
    X_d = traj{i};
    X_d_next = traj{i+1};

    %Updating Velocites using Feedforward and Feedback Controller.
    [u,X_err]=FeedbackControl(X,X_d,X_d_next,K_p,K_i,del_t,thetalist);
    velocities(6:9) = u(1:4);
    velocities(1:5) = u(5:9);

    % Getting Next State of the Bot using updated velociites
    next_state=Next_State(current_state,velocities,del_t,max_v);
    current_state = next_state;
    thetalist=transpose(current_state(4:8));

    % Updating Bot and End effector position w.r.t to {s} frame 
    Tsb = [[cos(current_state(1)), -sin(current_state(1)), 0, current_state(2)]; [sin(current_state(1)), cos(current_state(1)), 0, current_state(3)]; [0, 0, 1, 0.0963]; [0, 0, 0, 1]];
    T_s0 = Tsb*T_b0;
    T_0e = FKinBody(M_0e, Blist, transpose(current_state(4:8)));
    X = T_s0*T_0e;

    %storing current state and Xerr for writing into CSV
    current_state_append = [current_state_append; current_state];
    X_err_append = [X_err_append,X_err];
    
end

%-----Appending Gripper State to current state for CoppeliaSim Simulator---

%gripper state: intial to standoff above the block1%
gs_1 = zeros(1,500);
%gripper state: standoff_1 to block_1
gs_2 = zeros(1,500);
%gripper state: to grasp block_1%
gs_3 = ones(1,500);
%gripper state: to standoff1 after grasping block1%
gs_4 = ones(1,500);
%gripper state: to standoff2 from standoff1
gs_5 = ones(1,500);
%gripper state: block2 from standoff2
gs_6 = ones(1,500);
%gripper state: to drop block2
gs_7 = zeros(1,500);
%gripper state: back to standoff2
gs_8 = zeros(1,500);
%gripper state: standoff2 to initial
gs_9 = zeros(1,499);

gs = [gs_1,gs_2,gs_3,gs_4,gs_5,gs_6,gs_7,gs_8,gs_9];
current_state_append = [current_state_append , gs'];

writematrix(current_state_append, 'main_it1.csv');


%-----Error plot------------------------
plot(1:4499, X_err_append(1,:))
hold on
plot(1:4499, X_err_append(2,:))
hold on
plot(1:4499, X_err_append(3,:))
hold on
plot(1:4499, X_err_append(4,:))
hold on
plot(1:4499, X_err_append(5,:))
hold on
plot(1:4499, X_err_append(6,:))
hold off
ylabel('Error Twist Xerr')
xlabel('Time')
subtitle('K_P=6,K_I=4,Time scale=1/500th sec')
title('Error Convergence over Time')

writematrix(X_err_append,'error_it1.csv');
