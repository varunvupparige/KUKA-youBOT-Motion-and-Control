function[u,X_err]=FeedbackControl(X,X_d,X_dnext,KP,KI,del_t,thetalist)
%---------- MILESTONE 3 Feedforward Plus Feedback Control------------------%
% This function calculates the task-space feedforward plus feedback control law
% Inputs
% current actual end-effector configuration X , 
% current reference end-effector configuration Xd 
% reference end-effector configuration at the next timestep,Xd_next 
% The PI gain matrices Kp and Ki
% timestep Δt between reference trajectory configurations

%Output of FeedbackControl is the commanded end-effector twist V expressed in the end-effector frame {e}.
% X_err Error Twist is calculated to measure error across each joint. 

%---------------------Initilization (Constants)--------------------
M0e = [[1, 0, 0, 0.033]; [0, 1, 0, 0]; [0, 0, 1, 0.6546]; [0, 0, 0, 1]];

Tb0 = [[1, 0, 0, 0.1662]; [0, 1, 0, 0]; [0, 0, 1, 0.0026]; [0, 0, 0, 1]];

Blist = [[0; 0; 1;   0; 0.033; 0], ...
         [0; -1; 0;   -0.5076;   0;   0], ...
         [0; -1; 0;   -0.3526;   0;   0], ...
         [0; -1; 0; -0.2176; 0; 0], ...
         [0; 0; 1; 0; 0; 0]];

r = 0.0475;
w = 0.15;
l = 0.235;
%------------------------------------------------------------------

%---------------------Control law--------------------
F = (r/4)*[0 0 0 0; 0 0 0 0; -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1; 0 0 0 0];

adjoint=Adjoint(TransInv(X)*X_d);
vd=se3ToVec(MatrixLog6(TransInv(X_d)*X_dnext)/del_t);
feedforward=adjoint*vd;
X_err=se3ToVec(MatrixLog6(TransInv(X)*X_d));
P_ctrl= KP*X_err;
I_ctrl=KI*X_err*del_t;
vb=feedforward+P_ctrl+I_ctrl;
%---------------------------------------------------------------------

%-----------Calculating Wheel and Arm joint speeds---------------------
J_arm = JacobianBody(Blist, thetalist);
T0e = FKinBody(M0e, Blist, thetalist);
J_base= real(Adjoint(TransInv(T0e)*TransInv(Tb0)))*F;
Je_inv = pinv(real([J_base J_arm]),1e-3);
u=Je_inv*vb;
end