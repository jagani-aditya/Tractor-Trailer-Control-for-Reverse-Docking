function dState = myode(t, State, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function is the main component and represents
%   the single elements of the block diagram (open loop variant),
%   thereby putting all the functionality together.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Calculate reference values of x and y
[x_desired, eta, xdot_desired] = referenceTrajectory(t, Parameters);

%% Params
d0 = Parameters.d0;
d1 = Parameters.d1;
x0 = State(1);
y0 = State(2);
theta1 = State(4);

% output vector in terms of trailer coordinates
x1 = x0 - d1*cos(theta1);
y1 = y0 - d1*sin(theta1);
y_T = [ x1; y1];

%% steering Law calculate input reference values v and phi
v = stateFeedback(x_desired,State,y_T, Parameters);

%% Linear Feedback Controller using state feedback
% External function to get lie derivatives
LD = Truck_1T_LieDeriv(State(1:5), State(6:8), d0, d1);

A = [LD.L_g1_Lf3_h1, LD.L_g2_Lf3_h1;
    LD.L_g1_Lf3_h2, LD.L_g2_Lf3_h2];
c = [LD.Lf4_h1;
    LD.Lf4_h2];
b = v - c;
w = linsolve(A, b);

%% Controller State/Decoupling Controller in new parametrization (sigma)
xi_2 = State(7); 
xi_3 = State(8); 
w1 = w(1); 

% Compute controller state
dxi_1 = xi_2*eta*xdot_desired;
dxi_2 = xi_3*eta*xdot_desired;
dxi_3 = w1*eta*xdot_desired;

% Derivate of controller state vector
dx_C = [dxi_1 dxi_2 dxi_3]';

%% Reparametrization of the subject (change in time replacing the arc)
theta_0 = State(3); 
theta_1 = State(4);
xi_1 = State(6); 
w2 = w(2); 

% Define input vector elements u1 & u2 for Truck/Trailer model
u_1 = xi_1/cos(theta_0 - theta_1) * eta * xdot_desired; 
u_2 = w2 * eta * xdot_desired;

% Define input vector u for Truck/Trailer model
u = [u_1; u_2]; 

%% Simple vehicle model (time parametrized calculated state)
dx_T = myModel(State(1:5), u, Parameters);

%% Concatenate Truck and xi vector to State = [x_T; x_C]
State = [State(1:5); State(6:8)];
dState = [dx_T; dx_C];

end
