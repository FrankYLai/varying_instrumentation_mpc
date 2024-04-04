clear;
clc;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 1;

if configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
elseif configs("scene") == 2
    [obstacles, map, configs] = setup_scene2(configs);
else
    disp("please provide a scene description")
end


car  = setup_ego(configs);
car = car.set_wheel_velocity(1000,1000);

costs = [0];
% start simulation
tic;
dt = 0.04;
for t = 0:dt:10

    %update obstacle position
    car = car.update(dt);
    % car.x
    for i = 1:size(obstacles)+1
        obstacles(i) = obstacles(i).updatePose(t, configs);
    end
    
    %reference only collision logic
    % updateEgoPose(configs("obsList"),car.egoID,car.egoCapsule);
    % collisions = checkCollision(configs("obsList"))';
    cost = car.cost_dist(map, obstacles);
    [cross_section, collision_objs] = car.checkCollision(map, obstacles);
    if cross_section>0
        figure(2)
        plot(collision_objs)
        axis(configs("canvas"));
    end
    costs = [costs, cost];
    % if any(collisions)
    %     disp("collide")
    % end

    figure(1)
    animate(configs, car, obstacles, map, [], t);
    % figure(3)
    % plot(costs)

    %obstacle representation debugging purposes only
    % figure(2)
    % show(configs("obsList"));
    % axis(configs("canvas"));

end

toc