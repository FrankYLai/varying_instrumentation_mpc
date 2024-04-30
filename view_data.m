clear;
clc;
close all;


data = load('save_data/scene3_full_no_edge.mat');

show_animation(data)


function show_animation(data)
    dt = data.dt;
    final_time = data.t;
    configs = data.configs;
    car = setup_ego(configs);
    
    map = data.map;
    obstacles = data.obstacles;
    
    u_steps = data.save_u;
    path_steps = data.save_paths;
    
    curr_step = 1;
    for t = 0:dt:final_time
        for i = 1:size(obstacles,2)
            obstacles(i) = obstacles(i).updatePose(t);
        end
        u = u_steps(:,:,curr_step);
        car = car.step(u, car.mpc.dt, dt);
        
        path = path_steps(:,:,curr_step);
        figure(1)
        animate(configs, car, obstacles, map, path, t);
        title(configs("save_name"))
        curr_step = curr_step+1;
        pause(dt)
        
    end
end





