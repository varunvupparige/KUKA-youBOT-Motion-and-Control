
current_state1=[0,0,0,0,0,0,0,0,0,0,0,0];
current_state2=[0,0,0,0,0,0,0,0,0,0,0,0];
current_state3=[0,0,0,0,0,0,0,0,0,0,0,0];
velocities1=[0,0,0,0,0,10,10,10,10];
velocities2=[0,0,0,0,0,-10,10,-10,10];
velocities3=[0,0,0,0,0,-10,10,10,-10];
del_t=0.01;
store_states1=current_state1;
store_states2=current_state2;
store_states3=current_state3;
for i= 1:100
    next_state1=Next_State(current_state1,velocities1,del_t);
    current_state1=next_state1;
    store_states1=[store_states1;next_state1];

    next_state2=Next_State(current_state2,velocities2,del_t);
    current_state2=next_state2;
    store_states2=[store_states2;next_state2];

    next_state3=Next_State(current_state3,velocities3,del_t);
    current_state3=next_state3;
    store_states3=[store_states3;next_state3];
end
writematrix(store_states1,'Positive_X.csv');
writematrix(store_states2,'Positive_Y.csv');
writematrix(store_states3,'Spin_counterclockwise.csv');
