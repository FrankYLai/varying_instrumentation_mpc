clear;
clc;
close all;


data = load('save_data/scene1Full.mat');
figure(1)
plot([],[])
% pause(2)

show_animation(data, "scene1Full")

data = load('save_data/scene2Simple.mat');
figure(1)
plot([],[])
% pause(2)

show_animation(data, "scene1Simple")

data = load('save_data/scene1None.mat');
figure(1)
plot([],[])
% pause(2)

show_animation(data, "scene1None")
% plot_data(data)

% function plot_data(data)
%     u = data.save_u;
%     curr_node = 1;
%     final_time = data.t;
%     dt = data.dt;
%     u_current = u(1,:,:)
% 
% end

function show_animation(data, tit)
    dt = data.dt;
    final_time = data.t;
    configs = data.configs;
    car = setup_ego(configs);
    
    map = data.map;
    obstacles = data.obstacles;
    
    u_steps = data.save_u;
    path_steps = data.save_paths;
    
    
    car_dist_from_obs = [];
    curr_step = 1;
    for t = 0:dt:final_time
        
        dist_car = [];
        for i = 1:size(obstacles,2)
            obstacles(i) = obstacles(i).updatePose(t);
            dist_car(1,i) = norm(car.q(1:2)-obstacles(i).current_pose(1:2)) - ...
                            obstacles(i).dims(1);
        end
        
        car_dist_from_obs(curr_step,:) = dist_car;
        
        u = u_steps(:,:,curr_step);
        car = car.step(u, car.mpc.dt, dt);
        
        path = path_steps(:,:,curr_step);
        figure(1)
        animate(configs, car, obstacles, map, path, t);
        title(tit+" Instrumentation");
        curr_step = curr_step+1;
%         pause(dt)
        
    end
    figure(2)
    hold on;
    label = tit + 'min dist';
    min_dist = min(car_dist_from_obs,[],2)
    plot(0:dt:final_time, min_dist,'DisplayName', label)

    title("minimum robot distance to the obstacles")
    xlabel("time")
    ylabel("dist")
    legend
    
end





