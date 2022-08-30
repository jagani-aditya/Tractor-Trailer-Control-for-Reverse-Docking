function v = stateFeedback(Ref, State, y_T, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This function file describes the steering law and 
%   determines the input values (vector ny) for feedback 
%   linearization. Additionally, it implements a stabilization 
%   control by using the feedback of state and flat output. 
%   Therefore, the function takes the tracking error dynamics
%   into account. 
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Read weighting factors from struct 
k0 = Parameters.k0;
k1 = Parameters.k1;
k2 = Parameters.k2;
k3 = Parameters.k3;

% Read distances between axles from struct
d0 = Parameters.d0; 
d1 = Parameters.d1; 

% Split state vector in controller and vehicle state 
x_truck = State(1:5); 
x_ctrl = State(6:8); 

% Set references from CalcRefValues.m
xd = Ref.xRef; 
xd_dot = Ref.dxref_dsigma;
xd_ddot = Ref.d2xref_dsigma2;
xd_dddot = Ref.d3xref_dsigma3;
xd_ddddot = Ref.d4xref_dsigma4;

yd = Ref.yRef; 
yd_dot = Ref.dyref_dsigma;  
yd_ddot = Ref.d2yref_dsigma2;  
yd_dddot = Ref.d3yref_dsigma3;
yd_ddddot = Ref.d4yref_dsigma4;


%%
% Extract actual values from Truck_1T_LieDeriv.m
LD = Truck_1T_LieDeriv(x_truck, x_ctrl, d0,d1);

x_dot = LD.Lf_h1; 
x_ddot = LD.Lf2_h1; 
x_dddot = LD.Lf3_h1; 
y_dot = LD.Lf_h2; 
y_ddot = LD.Lf2_h2; 
y_dddot = LD.Lf3_h2; 

%%
% y(T) equals current Trailer position
x1 = y_T(1);    % x coordinate of the Trailer's rear axle
y1 = y_T(2);    % y coordinate of the Trailer's rear axle 

% e = k * [qd - q]
% Error Dynamics
% Difference between reference and actual values
ex = xd - x1;
ex_dot = xd_dot - x_dot;
ex_ddot = xd_ddot - x_ddot;
ex_dddot = xd_dddot - x_dddot;

ey = yd - y1;
ey_dot = yd_dot - y_dot;
ey_ddot = yd_ddot - y_ddot;
ey_dddot = yd_dddot - y_dddot;

% Equations of the controller for feedback linearization (input) 
v1 = xd_ddddot + k3*ex_dddot + k2*ex_ddot + k1*ex_dot + k0*ex; 
v2 = yd_ddddot + k3*ey_dddot + k2*ey_ddot + k1*ey_dot + k0*ey;

v = [v1; v2]; % Virtual control input

end

