function[next_state]=Next_State(current_state,velocities,del_t,max_v)
% ---------- MILESTONE 1 Kinematics Simulator (NextState)------------------%
% Using the kinematics of the youBot ,velocity kinematics and Euler method,
% we predict robot next state
% Inputs
% Current state of the robot (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles) current_state
% Joint and wheel velocities (9 variables: 5 for arm θ˙, 4 for wheels u) velocities
% Timestep size Δt (1 parameter) del_t
% Maximum joint and wheel velocity magnitude (1 parameter) max_v

% Outputs
% next state (configuration) of the robot (12 variables)

%---------------------Initilization (Constants)--------------------
r = 0.0475;
w = 0.15;
l = 0.235;
%--------------------------------------------------------------

F=(r/4)*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];

%Limiting Joint Speeds
for i=1:9
    if(abs(velocities(i))>max_v)
        velocities(i)= velocities(i)*max_v/abs(velocities(i));
    end
end

vb=F*transpose(velocities(6:9));
% Euler Method
next_state(4:8)=current_state(4:8)+velocities(1:5)*del_t;
next_state(9:12)=current_state(9:12)+velocities(6:9)*del_t;
% Youbot Configuration using Odometry
next_state(1:3)=current_state(1:3)+transpose(vb*del_t);
end

