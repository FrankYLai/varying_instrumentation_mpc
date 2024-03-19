clear;
clc;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 1;

if configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
end

Car  = setup_ego(configs);

% start simulation
tic;
while toc<40

    %update obstacle position
    for i = 1:size(obstacles)+1
        obstacles(i) = obstacles(i).updatePose(toc, configs);
    end
    
    %reference only collision logic
    states = states + [toc*0.2,0,0];
    egoCapsule1.States = states;
    updateEgoPose(configs("obsList"),1,egoCapsule1);
    collisions = checkCollision(configs("obsList"))';
    if any(collisions)
        disp("collide")
    end

%     figure(1)
%     animate(configs, [], obstacles, map, [], toc);

    %obstacle representation debugging purposes only
    figure(2)
    show(configs("obsList"));
    axis(configs("canvas"));
end

% for t = 1:0.1:10
%     animate(configs, [], obstacles, map, [], t);
% end