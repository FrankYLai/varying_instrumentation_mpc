clear;
clc;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 0;


if configs("scene") == 0
    [obstacles, map, configs] = setup_scene0(configs);
elseif configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
elseif configs("scene") == 2
    [obstacles, map, configs] = setup_scene2(configs);
else
    disp("please provide a scene description")
end


car  = setup_ego(configs);

costs = [0];
% start simulation
tic;
dt = 0.04;
for t = 0:dt:10000
    %update obstacle position

    for i = 1:size(obstacles,2)
        obstacles(i) = obstacles(i).updatePose(t);
    end
    
    %reference only collision logic
    cost = car.cost_dist(map, obstacles);
    % [cross_section, collision_objs] = car.checkCollision(map, obstacles);
    [raccel, laccel] = keyboard_accel_control();
    car = car.step(raccel, laccel, dt, dt);
    % [raccel, laccel, states] = car.optimize(configs('end'), map, obstacles);
    % if cross_section>0
    %     % figure(2)
    %     % plot(collision_objs)
    %     % axis(configs("canvas"));
    %     disp("collision detected");
    % end
    % costs = [costs, cost];
    % figure(3)
    % plot(costs)

    figure(1)
    animate(configs, car, obstacles, map, [], t);

    %obstacle representation debugging purposes only
    % figure(2)
    % show(configs("obsList"));
    % axis(configs("canvas"));

end

toc
