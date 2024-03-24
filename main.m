clear;
clc;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 1;

if configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
end

car  = setup_ego(configs);
car = car.set_wheel_velocity(1000,1000);


% start simulation
tic;
dt = 0.05;
for t = 0:dt:10

    %update obstacle position
    car = car.update(dt);
    % car.x
    for i = 1:size(obstacles)+1
        obstacles(i) = obstacles(i).updatePose(t, configs);
    end
    
    %reference only collision logic
    updateEgoPose(configs("obsList"),car.egoID,car.egoCapsule);
    collisions = checkCollision(configs("obsList"))';
    if any(collisions)
        disp("collide")
    end

    % figure(1)
    % animate(configs, car, obstacles, map, [], t);

    % figure(2)
    % show(configs("obsList"));
    % axis(configs("canvas"));
    %obstacle representation debugging purposes only
%     states = states + [toc*0.2,0,0];
%     egoCapsule1.States = states;
%     updateEgoPose(configs("obsList"),1,egoCapsule1);
%     figure(2)
%     show(configs("obsList"));
%     axis(configs("canvas"));
end

toc