function dx_T = myModel(x_T, u, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function describes the model of the Truck/Trailer 
%   system including x-/y-coordinates, orientation and steering
%   angles and is the plant of the control system. 
%   Its structure can be seen in the following equation: 
%       dx_T = f(x_T, u).
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize necessary values for formula implementation
d0 = Parameters.d0;
d1 = Parameters.d1;
theta0 = x_T(3); 
theta1 = x_T(4);
phi = x_T(5); 
u1 = u(1); 
u2 = u(2);

% Define elements of the Truck/Trailer vector (vehicle model)
dx_0 = u1*cos(theta0); 
dy_0 = u1*sin(theta0); 
theta0_dot = u1/d0 * tan(phi); 
theta1_dot = u1/d1 * sin(theta0 - theta1);
phi_dot = u2; 

% Define Truck/Trailer vector (vehicle model)
dx_T = [dx_0; 
        dy_0; 
        theta0_dot; 
        theta1_dot; 
        phi_dot]; 

end


