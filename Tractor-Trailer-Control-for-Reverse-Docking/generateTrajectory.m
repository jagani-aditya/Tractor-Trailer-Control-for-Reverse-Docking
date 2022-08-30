function a = generateTrajectory(XStart, XFinal)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Simple Path Planning Function regarding Truck Trailer System
%   state_x0 = start pose
%   state_x1 = end pose
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clarify elements of the state vectors
% state_x0 = [x1_s, y1_s, theta0_s, theta1_s, phi_s];
% state_x1 = [x1_e, y1_e, theta0_e, theta1_e, phi_e];

%% Inititalize intital state vector - start pose
x_s = XStart(1);
y_s = XStart(2);
theta0_s = XStart(3);
theta1_s = XStart(4);
phi_s = XStart(4);

% Inititalize final state vector - end pose
x_f = XFinal(1);
y_f = XFinal(2);
theta0_f = XFinal(3);
theta1_f = XFinal(4);
phi_f = XFinal(4);

%% Reference values (initial and final pose)
q0 = [0 0 0 0]; % x y x_dot y_dot for initial 
qf = [30 30 0 0]; % x y x_dot y_dot for final 

%% Determine the Coefficents of the polynomial by solving the LGS (Ax = b) 
% Concatenate to vector b (refers to both start and end pose)
q = [q0, qf]'; 

% Matrix (containing 8 conditions)
A = [x_s^7,    x_s^6,     x_s^5,    x_s^4,    x_s^3,   x_s^2,   x_s,  1;
    7*x_s^6,   6*x_s^5,   5*x_s^4,  4*x_s^3,  3*x_s^2, 2*x_s,   1 ,    0;
    42*x_s^5,  30*x_s^4,  20*x_s^3, 12*x_s^2, 6*x_s^1, 2,        0,     0;
    210*x_s^4, 120*x_s^3, 60*x_s^2, 24*x_s^1, 6,        0,        0,     0;
    x_f^7,     x_f^6,     x_f^5,    x_f^4,    x_f^3,   x_f^2,   x_f,  1;
    7*x_f^6,   6*x_f^5,   5*x_f^4,  4*x_f^3,  3*x_f^2, 2*x_f,   1 ,    0;
    42*x_f^5,  30*x_f^4,  20*x_f^3, 12*x_f^2, 6*x_f^1, 2,        0,     0;
    210*x_f^4, 120*x_f^3, 60*x_f^2, 24*x_f^1, 6,        0,        0,     0];

% A = [x1_s^7,    x1_s^6,     x1_s^5,    x1_s^4,    x1_s^3,   x1_s^2,   x1_s,  1;
%     7*x1_s^6,   6*x1_s^5,   5*x1_s^4,  4*x1_s^3,  3*x1_s^2, 2*x1_s,   1 ,    0;
%     42*x1_s^5,  30*x1_s^4,  20*x1_s^3, 12*x1_s^2, 6*x1_s^1, 2,        0,     0;
%     210*x1_s^4, 120*x1_s^3, 60*x1_s^2, 24*x1_s^1, 6,        0,        0,     0;
%     840*x1_s^3, 360*x1_s^2, 120*x1_s,  24,        0         0,        0,     0;  
%     x1_e^7,     x1_e^6,     x1_e^5,    x1_e^4,    x1_e^3,   x1_e^2,   x1_e,  1;
%     7*x1_e^6,   6*x1_e^5,   5*x1_e^4,  4*x1_e^3,  3*x1_e^2, 2*x1_e,   1 ,    0;
%     42*x1_e^5,  30*x1_e^4,  20*x1_e^3, 12*x1_e^2, 6*x1_e^1, 2,        0,     0;
%     210*x1_e^4, 120*x1_e^3, 60*x1_e^2, 24*x1_e^1, 6,        0,        0,     0;
%     840*x1_e^3, 360*x1_e^2, 120*x1_e,  24,        0         0,        0,     0];



% Output (solving the LGS via linsolve) 
a = linsolve(A, q);

end