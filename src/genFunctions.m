% genFunctions.m
clc; clear; close all;
% Retrieves our parameters
p = getParams;
% Define our key components of the optimization
% Decision vector
% Need to know the number of nodes
N = p.N;
m = p.m;
% Decision vector
F = sym('F',[N 1]); % Input forces
tor = sym('tor',[N 1]); % Input forces
x = sym('x',[N 1]); % Displacement
dx = sym('dx',[N 1]); % Velocity
y = sym('y',[N 1]); % Displacement
dy = sym('dy',[N 1]); % Velocity
th = sym('th',[N 1]); % Displacement
dth = sym('dth',[N 1]); % Velocity
w = [F;tor;x;dx;y;dy;th;dth];
current_state = [x;dx;y;dy;th;dth];
r = sqrt((xe-x).^2+(ye-y).^2); % hypotenuse
Fg = G*m*me./(r.^2); % Universal law of gravitation
% Cost function
syms qfinal % Defines our final position
cost = (p.T/(N-1))*sum((current_state-qfinal).^2);
% Constraints
% defect constraints
h = p.T/(N-1);
defect_pos_x = x(2:end) - x(1:end-1) - dx(1:end-1)*h;
defect_pos_y = y(2:end) - y(1:end-1) - dy(1:end-1)*h;
defect_pos_th = th(2:end) - th(1:end-1) - dth(1:end-1)*h;
defect_vel_x = dx(2:end) - dx(1:end-1) - (1/m)*(-Fg(1:end-1).*x(1:end-1)./r(1:end-1) ...
    - F(1:end-1).*cos(th(1:end-1)))*h;
defect_vel_y = dy(2:end) - dy(1:end-1) - (1/m)*(-Fg(1:end-1).*x(1:end-1)./r(1:end-1)...
    + F(1:end-1).*sin(th(1:end-1)))*h;

% r_numeric = double(subs(r(1:end-1), [F; x; y; th], zeros(size([F; x; y; th]))));
% % Replace any potential zeros with a small positive value (avoid division by zero)
% r_numeric = r_numeric + (r_numeric == 0) * eps;
% % Use the numeric expression in the computation
% defect_vel_x = dx(2:end) - dx(1:end-1) - (1/m)*(-Fg(1:end-1).*x(1:end-1)./r_numeric ...
%     - F(1:end-1).*cos(th(1:end-1)))*h;
% defect_vel_y = dy(2:end) - dy(1:end-1) - (1/m)*(-Fg(1:end-1).*x(1:end-1)./r_numeric ...
%     - F(1:end-1).*sin(th(1:end-1)))*h;

defect_vel_th = dth(2:end) - dth(1:end-1) - (tor(1:end-1)/I)*h;
defect = [defect_pos_x; defect_vel_x;
    defect_pos_y; defect_vel_y;
    defect_pos_th; defect_vel_th];
% initial conditions (where are we right now)
syms x0 dx0 y0 dy0 th0 dth0
init_cond = [x(1) - x0; dx(1) - dx0;
    y(1) - y0; dy(1) - dy0;
    th(1) - th0; dth(1) - dth0];
% inequality saturation limits
% Aw <= b
limits = [F-p.Fmax; -F+p.Fmin;
    tor-p.Tormin; -tor+p.Tormin];
% Equality constraints
% C(w) = 0
Ceq = [defect; init_cond]; % Only equality constraints are our defects
% Inequality constraints
Cineq = limits; % Only inequality constraints are
% our saturation limits
% Use the symbolic toolbox to generate functions
% that give us Hessians, gradients, constraints, etc.
H = hessian(cost,w);
c = gradient(cost,w);
c = subs(c,w,zeros(size(w))); % gets rid of decision vars
Aeq = jacobian(Ceq,w);
% beq = -subs(Ceq,w,ones(size(w)));
beq = -subs(Ceq,w,zeros(size(w)));
A = jacobian(Cineq,w);
b = -subs(Cineq,w,zeros(size(w)));
x0 = [s0; ds0];
matlabFunction(H,'File','Hfunc','Vars',{x0,sfinal})
matlabFunction(c,'File','cfunc','Vars',{x0,sfinal})
matlabFunction(Aeq,'File','Aeqfunc','Vars',{x0,sfinal})
matlabFunction(beq,'File','beqfunc','Vars',{x0,sfinal})
matlabFunction(A,'File','Afunc','Vars',{x0,sfinal})
matlabFunction(b,'File','bfunc','Vars',{x0,sfinal})