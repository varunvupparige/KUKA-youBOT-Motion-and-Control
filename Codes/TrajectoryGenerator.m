function[traj]=TrajectoryGenerator()

%------- MILESTONE 2  Reference trajectory generation --------------%
%This funtion create the reference (desired) trajectory for the end-effector frame {e}.
% Trajectory consists of eight concatenated trajectory segments, described below.

%1. Move the gripper from its initial configuration to a "standoff” configuration a few cm above the block.
%2. Move the gripper down to the grasp position.
%3. Close the gripper2.
%4. Move the gripper back up to the "standoff” configuration.
%5. Move the gripper to a "standoff” configuration above the final configuration.
%6. Move the gripper to the final configuration of the object.
%7. Open the gripper.
%8. Move the gripper back to the "standoff” configuration

Tf = 5;
N = 500;
method = 5;


%Xinitial = Tse
Xinitial = [0 0 1 0;0 1 0 0;-1 0 0 0.5;0 0 0 1];  
% Cube Initial (1,0)
% Xstandoff_1=Tsc_initial*Tce_standoff
Xstandoff_1 = [-0.7071 0 0.7071 1;0 1 0 0;-0.7071 0 -0.7071 0.125;0 0 0 1]; 
% Xblock1 =Tsc_initial*Tce_grasp
Xblock1 = [-0.7071 0 0.7071 1;0 1 0 0;-0.7071 0 -0.7071 0.025;0 0 0 1]; 

%Cube Final (0,-1)
% Xstandoff_2=Tsc_final*Tce_standoff
Xstandoff_2 = [0 1 0 0;0.7071 0 -0.7071 -1;-0.7071 0 -0.7071 0.125;0 0 0 1];
% Xblock2=Tsc_final*Tce_grasp
Xblock2 = [0 1 0 0;0.7071 0 -0.7071 -1;-0.7071 0 -0.7071 0.025;0 0 0 1];


%-------------------New_State Case-------------------------------
%Uncomment to run the New_state case
%{
%Cube Initial (1,1)
% Xstandoff_1=Tsc_final*Tce_standoff
Xstandoff_1 = [-0.7071 0 0.7071 1;0 1 0 1;-0.7071 0 -0.7071 0.125;0 0 0 1];
% Xblock1=Tsc_final*Tce_grasp
Xblock1 = [-0.7071 0 0.7071 1;0 1 0 1;-0.7071 0 -0.7071 0.025;0 0 0 1];

% Cube Final (1,-1)
% Xstandoff_2=Tsc_initial*Tce_standoff
Xstandoff_2 = [0 1 0 1; 0.7071 0 -0.7071 -1;-0.7071 0 -0.7071 0.125;0 0 0 1]; 
% Xblock2 =Tsc_initial*Tce_grasp
Xblock2 = [0 1 0 1; 0.7071 0 -0.7071 -1;-0.7071 0 -0.7071 0.025;0 0 0 1]; 
%}
%---------------------------------------------------------


%Trajectory intial to standoff above the block1%
traj_1 = ScrewTrajectory(Xinitial,Xstandoff_1,Tf,N,method);
%Trajectory standoff_1 to block_1
traj_2 = ScrewTrajectory(Xstandoff_1,Xblock1,Tf,N,method);
%Trajectory to grasp block_1%
traj_3 = ScrewTrajectory(Xblock1,Xblock1,Tf,N,method);
%Trajectory to standoff1 after grasping block1%
traj_4 = ScrewTrajectory(Xblock1,Xstandoff_1,Tf,N,method);
%Trajectory to standoff2 from standoff1
traj_5 = ScrewTrajectory(Xstandoff_1,Xstandoff_2,Tf,N,method);
%Trajectory to block2 from standoff2
traj_6 = ScrewTrajectory(Xstandoff_2,Xblock2,Tf,N,method);
%Trajectory to drop block2
traj_7 = ScrewTrajectory(Xblock2,Xblock2,Tf,N,method);
%Trajectory from block2 to standoff_2
traj_8 = ScrewTrajectory(Xblock2,Xstandoff_2,Tf,N,method);
%Trajectory from standoff_2 to initial 
traj_9 = ScrewTrajectory(Xstandoff_2,Xinitial,Tf,N,method);
traj = [traj_1,traj_2,traj_3,traj_4,traj_5,traj_6,traj_7,traj_8,traj_9];

end