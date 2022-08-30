% Clear former data
clear
close
clc

%% Variables
T = 10; % Define travel time with interface input - predefined with 10s
d0 = 2; % Distance between front and rear axle of the truck
d1 = 3; % Distance between truck's rear axle and trailer's axle
input_start = [0 0 0 0 0 ]; % x y theta_0 theta_1 phi [in degree]
input_final = [30 30 0 0 0]; % x y theta_0 theta_1 phi [in degree]
Px = 0.2; % Deviation from predefined x-coordinate
Py = 0.5; % Deviation from predefined y-coordinate

%% Initial/start pose

start.x1 = input_start(1);               % X-Coordinate of Trailer's rear axle
start.y1 = input_start(2);               % Y-Coordinate of Trailer's rear axle
start.theta0 = rad2deg(input_start(3));  % Orientation angle of Truck in degree
start.theta1 = rad2deg(input_start(4));  % Orientation angle of Trailer in degree
start.phi = rad2deg(input_start(5));     % Steering angle of Truck in degree
start.x0 = start.x1 + d1*cos(start.theta1); % X-Coordinate of Truck's rear axle
start.y0 = start.y1 + d1*sin(start.theta1); % Y-Coordinate of Truck's rear axle

XStart = [start.x1, start.y1, start.theta0, start.theta1, start.phi];

%% Final/end pose

final.x1 = input_final(1);   % X-Coordinate of Trailer's rear axle
final.y1 = input_final(2);   % Y-Coordinate of Trailer's rear axle
final.theta0 = rad2deg(input_final(3));  % Orientation angle of Truck in degree
final.theta1 = rad2deg(input_final(4));  % Orientation angle of Trailer in degree
final.phi = rad2deg(input_final(5));     % Steering angle of Truck in degree
final.x0 = final.x1 + d1*cos(final.theta1); % X-Coordinate of Truck's rear axle
final.y0 = final.y1 + d1*sin(final.theta1); % Y-Coordinate of Truck's rear axle

% Build the vectors for initial and final pose of the vehicle (trailer)
XFinal = [final.x1, final.y1, final.theta0, final.theta1, final.phi];

%% Generate Reference trajectory
coef = generateTrajectory(XStart, XFinal);

%% Weighting factors for state feedback based on Hurwitz criterion
k0 = 0.0625; 
k1 = 0.5; 
k2 = 0.75;  
k3 = 2; 

% change sign of k1 & k3 in case of backward motion
k1 = k1 * -1;
k3 = k3 * -1;

% Define a set of parameters containing 
Parameters.coef = coef;             % Polynomial coeffients
Parameters.d0 =   d0;               % Axle distances (Truck)
Parameters.d1 =   d1;               % Axle distances (hitch length)
Parameters.T =    T;                 % Travel time
Parameters.XStart = start.x1;         % Start pose of Truck/Trailer System
Parameters.XFinal = final.x1;         % Final pose of Truck/Trailer System
Parameters.k0 = k0;               % Weighting factors (state feedback) 
Parameters.k1 = k1; 
Parameters.k2 = k2; 
Parameters.k3 = k3; 

% B Matrix controller states
xi_1 = 1;   
xi_2 = 0;
xi_3 = 0;

%% Trajectory calculation of stabilizing feedback control for Truck/Trailer system
% Reference trajectory for plotting tests 
% Auxiliary graph for comparison with the resulting trajectory
i=0;
xRef = zeros(1,T/0.01);
yRef = zeros(1,T/0.01);
for t=0:0.01:T
    i=i+1;
    [Ref(i), ~, ~] = referenceTrajectory(t, Parameters);
    xRef(i) = Ref(i).xRef;
    yRef(i) = Ref(i).yRef;
end

%% ODE

% Start initial position of trailer
x0 = final.x1 - Px;
y0 = final.y1 - Px;
theta0 = final.theta0;
theta1 = final.theta1;
phi = final.phi;
XStart = [x0, y0, theta0, theta1, phi, xi_1, xi_2, xi_3];

[t, State] = ode45(@myode, [0,T], XStart, [], Parameters); 

%% Simulating and Plotting Trajectory
% Determine the Trucks's position based on State vector
for i=1:length(t)
    x0(i) = State(i,1);
    y0(i) = State(i,2);
end

% Determine the Trailer's position based on the Truck's coordinates
for i=1:length(t)
    x1(i) = State(i,1) - d1*cos(State(i,4));
    y1(i) = State(i,2) - d1*sin(State(i,4));
end

% Plot Reference Trajectory vs Resulting Trajectories
figure; 
plot(y0, x0, 'Color', 'r'); % Resulting trajectory of Truck/Trailer System (Truck perspective)
hold on
plot(y1, x1, 'Color', 'b'); % Resulting trajectory of Truck/Trailer System (Trailer perspective)
hold on
plot(yRef, xRef, 'Color', [0 0.5 0]); % Reference trajectory of Truck/Trailer System
title('Reference Trajectory vs Resulting Trajectories')
xlabel('x-coordinate')
ylabel('y-coordinate')
legend('Resulting Trajectory Truck', 'Resulting Trajectory Trailer', 'Reference Trajectory', 'Location', 'northeastoutside')
grid on;
grid minor;



