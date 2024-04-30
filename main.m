clear;
clc;
close all;

addpath(genpath(pwd));

configs = containers.Map;
configs("scene") = 2;
configs("tracking") = "none";
configs("save_name") = "scene2None.mat";


if configs("scene") == 0
    [obstacles, map, configs] = setup_scene0(configs);
elseif configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
elseif configs("scene") == 2
    [obstacles, map, configs] = setup_scene2(configs);
elseif configs("scene") == 3
    [obstacles, map, configs] = setup_scene3(configs);
else
    disp("please provide a scene description")
end


car  = setup_ego(configs);

costs = [0];
% start simulation
tic;
dt = 0.1;
t = 0;
step = true;

save_paths = [];
save_u = [];
save_pose = [];
b = 0


while not_completed(car, configs("end"), map, obstacles)
    t
    %update obstacle position

    for i = 1:size(obstacles,2)
        obstacles(i) = obstacles(i).updatePose(t);
    end
    
    %update car position
    [u, path] = car.optimize(configs('end'), map, obstacles);
    
    if step
        car = car.step(u, car.mpc.dt, dt);
    end

    save_paths(:,:,end+1) = path;
    save_u(:,:,end+1) = u;
    save_pose(:,:,end+1) = car.q;


    figure(1)
    animate(configs, car, obstacles, map, path, t);

    t = t+dt;
    if b
        break
    end

end

toc

save(configs("save_name"));


function ncomp = not_completed(car, end_pos, map, obstacles)
    ncomp = norm(car.q(1:2)-end_pos)>0.3;
    obs = [map, obstacles];
%     for o = obs
%         if norm(car.q(1:2) - o.current_pose(1:2))< o.dims(1)+car.b
%             disp("collision detected")
%             ncomp = false;
%         end
%     end
end

    
    
    