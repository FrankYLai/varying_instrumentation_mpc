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

     %% MPC Stuff
    for t_current = 0:Ts:tfinal
        % Run our trajectory optimization
        wstar = quadprog(Hfunc(x0,qfinal), cfunc(x0,qfinal), ...
            Afunc(x0,qfinal), bfunc(x0,qfinal), ...
            Aeqfunc(x0,qfinal), beqfunc(x0,qfinal));
        % Grabs the first control input from u(t)
        F = wstar(1);
        % Run simulation for Ts seconds with our optimized
        % control input, F
        %     dyn2 = @(t,x,F) [x(2); F/p.m]; % Defines our dynamics
        %     dynamics = @(t,x) dyn2(t,x,F);
        dyn = @(t,q) dynamics(t,q,u);
        %     [tout, xout] = ode45(dyn,[0 Ts],x0); % Simulation
        [tout, qout] = ode45(dyn, t_span, q0);
        % Store times, states
        t_store = [t_store; tout+t_current];
        x_store = [x_store; xout];
        u_store = [u_store; ones(size(tout))*F];
        % Resample our state
        x0 = xout(end,:).';
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
