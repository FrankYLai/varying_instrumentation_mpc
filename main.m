clear;
clc;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 2;

if configs("scene") == 1
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
    car = car.update(dt);
    % car.x
    for i = 1:size(obstacles)+1
        obstacles(i) = obstacles(i).updatePose(t);
    end
    
    %reference only collision logic
    cost = car.cost_dist(map, obstacles);
    [cross_section, collision_objs] = car.checkCollision(map, obstacles);
    [accel, angular_accel] = keyboard_accel_control();
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


    car = car.control_wheel_acceleration(accel, angular_accel, dt);

    %obstacle representation debugging purposes only
    % figure(2)
    % show(configs("obsList"));
    % axis(configs("canvas"));

end

toc