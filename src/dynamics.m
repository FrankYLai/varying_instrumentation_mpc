function [dq] = dynamics(t,q,u)
p = getParams();
% Store parameters
m = p.m;
r = p.r;
L = p.L;
I = p.I;
x = q(1,:);
dx = q(2,:);
y = q(3,:);
dy = q(4,:);
th = q(5,:);
dth = q(6,:);
phi_dot_L = q(7,:);
phi_dot_R = q(8,:);
phi_ddot_L = u(1,:);
phi_ddot_R = u(2,:);
% Define our derivatives
dq = zeros(size(q)); % Pre-allocating
dq(1,:) = r/2*(phi_dot_L + phi_dot_R)*cos(th);
dq(2,:) = r/2*(phi_dot_L + phi_dot_R)*sin(th);
dq(3,:) = r/L*(phi_dot_L + phi_dot_R);
dq(4,:) = phi_ddot_L;
dq(5,:) = phi_ddot_R;