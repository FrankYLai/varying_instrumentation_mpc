% getParams.m
function p = getParams()
p.N = 21; % Number of Nodes
p.m = 1; % Mass (kg)
p.Fmin = -300; % Minimum force (N)
p.Fmax = 300; % Maximum force (N)
p.T = 1; % Time horizon (s)
p.Ts = 0.1;
p.r = .1; %radius of wheels
p.L = 1; %distance between wheels