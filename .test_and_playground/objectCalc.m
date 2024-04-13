% Define parameters
sigma = 1; % Standard deviation (still looking up what is needed)
A = 1; % Scaling factor (not sure what we want here)
x1 = 0; % Center of obstacle 1 in x
y1 = 0; % Center of obstacle 1 in y
x2 = 1; % Center of obstacle 2 in x
y2 = 1; % Center of obstacle 2 in y
x3 = 2; % Center of obstacle 3 in x
y3 = 2; % Center of obstacle 3 in y
robotPoints = [3,3]; % car center point


% Define workspace grid (this is just rough rn, i havent finising reading
% through your code yet)
[X, Y] = meshgrid(-5:0.1:5, -5:0.1:5);

% Compute repulsive force for both obstacles using Gaussian function, this
% is done over the entire enviroment to find the field force at a specific
% location, need the robot center (doesnt account for size of robot)
F1 = A * exp(-((X - x1).^2 + (Y - y1).^2) / (2 * sigma^2));
F2 = A * exp(-((X - x2).^2 + (Y - y2).^2) / (2 * sigma^2));
F3 = A * exp(-((X - x3).^2 + (Y - y3).^2) / (2 * sigma^2));


% Visualize the potential field, can use this in the display?
figure;
contour(X, Y, F1);
hold on
contour (X, Y, F2);
hold on
contour (X, Y, F3);
xlabel('X');
ylabel('Y');
title('Repulsive Potential Field');
colorbar;