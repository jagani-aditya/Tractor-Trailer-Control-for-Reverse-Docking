function [Ref, eta, qd_dot] = referenceTrajectory(t, Parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Calculation of the reference values regarding the Truck's
%   coordinates (x, y). 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extraction of all necessary parameters
a = Parameters.coef(1);
b = Parameters.coef(2);
c = Parameters.coef(3);
d = Parameters.coef(4);
e = Parameters.coef(5);
f = Parameters.coef(6);
g = Parameters.coef(7);
h = Parameters.coef(8);

X0 = Parameters.XStart;
Xf = Parameters.XFinal; 
T = Parameters.T; 

tau = (T - t)/T; 

% Defining a scaling parameter and its first derivative
s_tau = 3*tau^2 - 2*tau^3; 
ds_tau = 6*tau - 6*tau^2;  

% Possible time parametrization of x and its first derivative
qd = X0 + (Xf-X0)*s_tau; 
qd_dot = -1/T*(Xf-X0)*ds_tau; % x dot needed as input for controller state

% Polynomial 
xd =   a*qd^7     + b*qd^6     + c*qd^5     + d*qd^4    + e*qd^3   + f*qd^2   + g*qd^1 + h;
xd_dot =    7*a*qd^6   + 6*b*qd^5   + 5*c*qd^4   + 4*d*qd^3  + 3*e*qd^2 + 2*f*qd^1 + g; 
xd_ddot =   42*a*qd^5  + 30*b*qd^4  + 20*c*qd^3  + 12*d*qd^2 + 6*e*qd^1 + 2*f;
xd_dddot =  210*a*qd^4 + 120*b*qd^3 + 60*c*qd^2  + 24*d*qd^1 + 6*e;
xd_ddddot = 840*a*qd^3 + 360*b*qd^2 + 120*c*qd^1 + 24*d;

%%
% System model parametized by arc length to remove time dependency
% Determine eta
eta = sqrt(1 + xd_dot^2); 

% Determine the ref values derived according to sigma (arc length) 
dxref_dsigma = 1/eta; 
d2xref_dsigma2 = -(xd_dot*xd_ddot)/(eta^4);
d3xref_dsigma3 = -((xd_ddot^2) + xd_dot*xd_dddot) / (eta^5) + (4*(xd_dot^2)*(xd_ddot^2)) / (eta^7); 
d4xref_dsigma4 = -( (2*xd_ddot*xd_dddot + xd_ddot*xd_dddot+ xd_dot*xd_ddddot)/eta^5 - 5/eta^6*xd_dot*xd_ddot/eta*(xd_ddot^2+xd_dot*xd_dddot))/eta ...
                 + ((8*xd_dot*xd_ddot^3+8*xd_dot^2*xd_ddot*xd_dddot)/eta^7 - 7/eta^8*xd_dot*xd_ddot/eta*4*xd_dot^2*xd_ddot^2 )/eta;


dyref_dsigma = xd_dot/eta; 
d2yref_dsigma2 = xd_ddot/(eta^2) - ((xd_dot^2)*xd_ddot)/(eta^4); 
d3yref_dsigma3 = xd_dddot/(eta^3) - (4*(xd_ddot^2)*xd_dot)/(eta^5) - ((xd_dot^2)*xd_dddot)/(eta^5) + (4*(xd_dot^3)*(xd_ddot^2))/(eta^7);
d4yref_dsigma4 = 1/eta*(xd_ddddot/eta^3 - 3*xd_dddot/eta^4*xd_dot*xd_ddot/eta - (8*xd_ddot*xd_dot*xd_dddot + 4*xd_ddot^3)/eta^5 + 20/eta^6*xd_ddot^3*xd_dot^2/eta ...
    - (2*xd_dot*xd_ddot*xd_dddot + xd_dot^2*xd_ddddot)/eta^5 + 5*xd_dot^3*xd_ddot*xd_dddot/eta^7 ...
    +(12*xd_dot^2*xd_ddot^3+ 8*xd_dot^3*xd_ddot*xd_dddot)/eta^7 - 28*xd_dot^4*xd_ddot^3/eta^9);

% Define struct for reference values
% We have two states x and y. Hence we need q1_desired, q1dot_desired,
% q1ddot_desired, 4th derivative for time dependency
% q1_desired = [qd qd_dot qd_ddot qd_dddot] (4th derivative to remove time
% dependency)
Ref.xRef = qd; 
Ref.dxref_dsigma = dxref_dsigma; 
Ref.d2xref_dsigma2 = d2xref_dsigma2;
Ref.d3xref_dsigma3 = d3xref_dsigma3; 
Ref.d4xref_dsigma4 = d4xref_dsigma4; 

% q2_desired = [qd qd_dot qd_ddot qd_dddot] (4th derivative to remove time
% dependency)
Ref.yRef = xd; 
Ref.dyref_dsigma = dyref_dsigma; 
Ref.d2yref_dsigma2 = d2yref_dsigma2; 
Ref.d3yref_dsigma3 = d3yref_dsigma3;
Ref.d4yref_dsigma4 = d4yref_dsigma4;

end
