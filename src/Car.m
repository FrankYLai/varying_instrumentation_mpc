classdef Car
    properties
        x       % x-position
        x_dot   % velocity
        wheel_speed
        b
        r
        car_dims
        car_points

        egoID

        MAX_WHEEL_ROT_SPEED_RAD = 2*pi; % Define MAX_WHEEL_ROT_SPEED_RAD
        MIN_WHEEL_ROT_SPEED_RAD = -2*pi; % Define MIN_WHEEL_ROT_SPEED_RAD
    end
    
    methods
        function obj = Car(initial_x, initial_y, initial_theta, initial_b, initial_r)
            obj.x = [initial_x; initial_y; initial_theta];
            obj.x_dot = [0; 0; 0];
            obj.wheel_speed = [0; 0];
            obj.b = initial_b;
            obj.r = initial_r;

            % triangular car
            obj.car_dims = [
                -obj.b, -obj.b, 1;
                0     , -obj.b, 1;
                obj.b,      0, 1;
                0,       obj.b, 1;
                -obj.b, obj.b, 1;

            ]; 

            obj = obj.get_transformed_pts();
            
            %car capsul
            obj.egoID = 1;
            
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
        
        function obj = get_transformed_pts(obj)
            rot_mat = [
                cos(obj.x(3)), sin(obj.x(3)), obj.x(1);
                -sin(obj.x(3)), cos(obj.x(3)), obj.x(2);
                0, 0, 1
            ];
            obj.car_points = obj.car_dims*rot_mat';
        end
        
        function points = get_points(obj)
            obj = obj.get_transformed_pts();
            points = obj.car_points;
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

        function [cross_section, collision_objs] = checkCollision(obj, static_obs, dynamic_obs)
            obs_arry = [static_obs,dynamic_obs];

            pts_ego = obj.get_points();
            poly_ego = polyshape(pts_ego(:,1), pts_ego(:,2));
            collision_objs = [];
            cross_section = 0;
            % pts_obs = obs_arry(1).gen_verticies(obs_arry(1).current_pose, obs_arry(1).dims);
            % poly_obs = polyshape(pts_obs(1,:), pts_obs(2,:));
            for obs = obs_arry
                pts_obs = obs.gen_verticies(obs.current_pose, obs.dims);
                poly_obs = polyshape(pts_obs(1,:), pts_obs(2,:));
                polyout = intersect(poly_ego,poly_obs);
                cross_section = cross_section + area(polyout);
                if area(polyout)>0
                    collision_objs =  polyout;
                end
            end
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