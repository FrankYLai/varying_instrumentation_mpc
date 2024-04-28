classdef Car
    properties
        x       % x-position
        x_dot   % velocity
        wheel_speed
        b
        r
        car_dims

        mpc
        MAX_WHEEL_ROT_SPEED_RAD = 4*pi; % Define MAX_WHEEL_ROT_SPEED_RAD
        MIN_WHEEL_ROT_SPEED_RAD = -4*pi; % Define MIN_WHEEL_ROT_SPEED_RAD

        MAX_WHEEL_ROT_ACCEL_RAD = 2*pi;
        MIN_WHEEL_ROT_ACCEL_RAD = -2*pi; % Define MIN_WHEEL_ROT_SPEED_RAD
    end
    
    methods
        function obj = Car(initial_x, initial_y, initial_theta, initial_b, initial_r)
            obj.x = [initial_x; initial_y; initial_theta];
            obj.x_dot = [0; 0; 0];
            obj.wheel_speed = [0; 0];
            obj.b = initial_b;
            obj.r = initial_r;

            % pentagon car
            obj.car_dims = [
                -obj.b, -obj.b, 1;
                0     , -obj.b, 1;
                obj.b,      0, 1;
                0,       obj.b, 1;
                -obj.b, obj.b, 1;

            ]; 
            
            lb = [-inf, -inf, -inf, obj.MIN_WHEEL_ROT_SPEED_RAD, obj.MIN_WHEEL_ROT_SPEED_RAD];
            ub = [inf, inf, inf, obj.MAX_WHEEL_ROT_SPEED_RAD, obj.MAX_WHEEL_ROT_SPEED_RAD];
            lb_u = [obj.MIN_WHEEL_ROT_ACCEL_RAD, obj.MIN_WHEEL_ROT_ACCEL_RAD];
            ub_u = [obj.MAX_WHEEL_ROT_ACCEL_RAD, obj.MAX_WHEEL_ROT_ACCEL_RAD];
            obj.mpc = MPC(lb, ub, lb_u, ub_u);   
        end
        
        function dq = dynamics(obj, t, q, u)
            % Store parameters
            th = q(5,:);
            phi_dot_L = q(7,:);
            phi_dot_R = q(8,:);
            phi_ddot_L = u(1,:);
            phi_ddot_R = u(2,:);
            % Define our derivatives
            dq = zeros(size(q)); % Pre-allocating
            dq(1,:) = obj.r/2*(phi_dot_L + phi_dot_R)*cos(th);
            dq(2,:) = obj.r/2*(phi_dot_L + phi_dot_R)*sin(th);
            dq(3,:) = obj.r/L*(phi_dot_L + phi_dot_R);
            dq(4,:) = phi_ddot_L;
            dq(5,:) = phi_ddot_R;
        end

        function obj = set_wheel_velocity(obj, lw_speed, rw_speed) %sets wheel speed for right and left wheels
            
            obj.wheel_speed = [
                rw_speed;
                lw_speed
            ];
            obj.x_dot = obj.forward_kinematics();
        end
        
        function obj = set_robot_velocity(obj, linear_velocity, angular_velocity)
            obj.x_dot = [
                linear_velocity;
                0;
                angular_velocity
            ];
            obj.wheel_speed = obj.inverse_kinematics();
        end
        
        function x = update_state(obj, dt)
            A = [
                1, 0, 0;
                0, 1, 0;
                0, 0, 1
            ];
            B = [
                sin(obj.x(3, 1) + pi/2)*dt,  0;
                cos(obj.x(3, 1) + pi/2)*dt,  0;
                0                        , dt
            ];
            vel = [
                obj.x_dot(1, 1);
                obj.x_dot(3, 1)
            ];
            x = A*obj.x + B*vel;
        end
        
        function obj = update(obj, dt)
            obj.wheel_speed(obj.wheel_speed > obj.MAX_WHEEL_ROT_SPEED_RAD) = obj.MAX_WHEEL_ROT_SPEED_RAD;
            obj.wheel_speed(obj.wheel_speed < obj.MIN_WHEEL_ROT_SPEED_RAD) = obj.MIN_WHEEL_ROT_SPEED_RAD;
            obj.x_dot = obj.forward_kinematics();
            obj.x = obj.update_state(dt);
            obj.wheel_speed = obj.inverse_kinematics();
        end
        
        function state = get_state(obj)
            state = {obj.x, obj.x_dot};
        end
        
        function kine = forward_kinematics(obj)
            kine_mat = [
                obj.r/2         , obj.r/2;
                0               , 0;
                obj.r/(2*obj.b), -obj.r/(2*obj.b)
            ];
            kine = kine_mat*obj.wheel_speed;
        end
        
        function ikine = inverse_kinematics(obj)
            ikine_mat = [
                1/obj.r, 0, obj.b/obj.r;
                1/obj.r, 0, -obj.b/obj.r
            ];
            ikine = ikine_mat*obj.x_dot;
        end
        
        function car_points = get_points(obj, X, Y, Theta)
            car_points = cell(numel(X),1);
            for i = 1:numel(X)
                x = X(i);
                y = Y(i);
                theta = Theta(i);
                rot_mat = [
                    cos(theta), sin(theta), x;
                    -sin(theta), cos(theta), y;
                    0, 0, 1
                ];
                car_points{i} = obj.car_dims*rot_mat';
            end


        end
        
        function obj = control_acceleration(obj, desired_acceleration, dt)
            current_velocity = obj.x_dot(1);
            if desired_acceleration >= 0
                obj.x_dot(1) = current_velocity + desired_acceleration * dt;
            else
                obj.x_dot(1) = max(0, current_velocity + desired_acceleration * dt); % Ensure velocity doesn't go negative
            end
        end
        
        function obj = control_wheel_acceleration(obj, left_wheel_acceleration, right_wheel_acceleration, dt)
            current_lw_speed = obj.wheel_speed(1);
            current_rw_speed = obj.wheel_speed(2);
            obj.wheel_speed(1) = current_lw_speed + left_wheel_acceleration * dt;
            obj.wheel_speed(2) = current_rw_speed + right_wheel_acceleration * dt;
        end

        function c = cost_dist(obj, static_obs, dynamic_obs)
            c = 0;
            for obs =  [static_obs,dynamic_obs]
                if obs.shape == "circle"
                    c = c+obj.cost_dist_single_circle(obj.x(1:2), obs.current_pose(1:2), obs.dims(1));
                else
                    obs_pose = obs.current_pose;
                    verticies = obs.gen_verticies(obs_pose, obs.dims);
                    c = c + obj.cost_dist_single(obj.x(1:2),verticies');
                end
            end
        end

        function [raccel, laccel, states] = optimize(obj, goal, map, obstacles)
            [u, states] = obj.mpc.optimize(obj, goal, map, obstacles);
            raccel = u(:,1);
            laccel = u(:,2);
        end




    end

    methods (Static)
        function c = cost_dist_single(pt, obs)
            mindist = 100;
            for k = 1:size(obs,1)  
                pt1 = obs(k,:);
                pt2 = obs(mod(k,size(obs,1))+1,:);
                d = point_to_line(pt, pt1, pt2);
                if d<mindist
                    mindist = d;
                end
            end
            % cost function
            c = 2/(mindist^2 + 1);
        
        end

        function c = cost_dist_single_circle(pt, center, r)
            pt = reshape(pt,1,2);
            v = pt-center;
            d = sqrt(sum(v.^2));
            
            c = 2/(d^2 + 1);

        end
    end
end


function dist = point_to_line(pt, v1, v2)
    pt = reshape(pt,1,2);
    a = v1 - v2;
    b = pt - v2;
    line_vec = a ;%vector(start, end) # (3.5, 0, -1.5)
    pnt_vec = b ;%vector(start, pnt)  # (1, 0, -1.5)
    line_len = sqrt(sum(line_vec.^2)); % # 3.808
    line_unitvec = line_vec/line_len; % # (0.919, 0.0, -0.394)
    pnt_vec_scaled = pnt_vec/line_len; %  # (0.263, 0.0, -0.393)
    t = dot(line_unitvec, pnt_vec_scaled); % # 0.397
    if t < 0.0
        t = 0.0;
    elseif t > 1.0
        t = 1.0;
    end
    nearest = line_vec* t; %    # (1.388, 0.0, -0.595)
    dist = sqrt(sum((nearest-pnt_vec).^2));% # 0.985
end