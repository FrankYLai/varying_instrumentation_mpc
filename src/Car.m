classdef Car
    properties
        x       % x-position
        y       % y-position
        theta   % heading angle
        x_dot   % velocity
        wheel_speed
        b
        r
        car_dims
        car_points
    end
    
    methods
        function obj = Car(initial_x, initial_y, initial_theta, initial_b, initial_r)
            obj.x = initial_x;
            obj.y = initial_y;
            obj.theta = initial_theta;
            obj.x_dot = [0; 0; 0];
            obj.wheel_speed = [0; 0];
            obj.b = initial_b;
            obj.r = initial_r;
            obj.car_dims = [
                -obj.b, -obj.b, 1;
                0     , -obj.b, 1;
                obj.b,      0, 1;
                0,       obj.b, 1;
                -obj.b, obj.b, 1
            ];
            obj.get_transformed_pts();
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
        
        function obj = update_state(obj, dt)
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
            obj.x = A*obj.x + B*vel;
        end
        
        function obj = update(obj, dt)
            MAX_WHEEL_ROT_SPEED_RAD = 0; % Define MAX_WHEEL_ROT_SPEED_RAD
            MIN_WHEEL_ROT_SPEED_RAD = 0; % Define MIN_WHEEL_ROT_SPEED_RAD
            obj.wheel_speed(obj.wheel_speed > MAX_WHEEL_ROT_SPEED_RAD) = MAX_WHEEL_ROT_SPEED_RAD;
            obj.wheel_speed(obj.wheel_speed < MIN_WHEEL_ROT_SPEED_RAD) = MIN_WHEEL_ROT_SPEED_RAD;
            obj.x_dot = obj.forward_kinematics();
            obj.update_state(dt);
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
                cos(obj.x), sin(obj.y), obj.theta;
                -sin(obj.x), cos(obj.y), obj.theta;
                0, 0, 1
            ];
            obj.car_points = obj.car_dims*rot_mat';
            obj.car_points = int64(obj.car_points);
        end
        
        function points = get_points(obj)
            obj.get_transformed_pts();
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
    end
end
