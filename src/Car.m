classdef Car
    properties
        q       % x-position, y-position, theta, phi_dot_L, phi_dot_R
        b
        r
        car_dims

        mpc
        MAX_WHEEL_ROT_SPEED_RAD = 4*pi; % Define MAX_WHEEL_ROT_SPEED_RAD
        MIN_WHEEL_ROT_SPEED_RAD = -4*pi; % Define MIN_WHEEL_ROT_SPEED_RAD

        MAX_WHEEL_ROT_ACCEL_RAD = 3*pi;
        MIN_WHEEL_ROT_ACCEL_RAD = -3*pi; % Define MIN_WHEEL_ROT_SPEED_RAD
    end
    
    methods
        function obj = Car(initial_x, initial_y, initial_theta, initial_b, initial_r)
            obj.q = [initial_x, initial_y, initial_theta, 0, 0];
            obj.b = initial_b;
            obj.r = initial_r;

            % % pentagon car
            obj.car_dims = [
                -obj.b, -obj.b, 1;
                0     , -obj.b, 1;
                obj.b,      0, 1;
                0,       obj.b, 1;
                -obj.b, obj.b, 1;

            ]; 
            % obj.car_dims = [
            %     obj.b
            % ]
            
            lb = [-inf, -inf, -inf, obj.MIN_WHEEL_ROT_SPEED_RAD, obj.MIN_WHEEL_ROT_SPEED_RAD];
            ub = [inf, inf, inf, obj.MAX_WHEEL_ROT_SPEED_RAD, obj.MAX_WHEEL_ROT_SPEED_RAD];
            lb_u = [obj.MIN_WHEEL_ROT_ACCEL_RAD, obj.MIN_WHEEL_ROT_ACCEL_RAD];
            ub_u = [obj.MAX_WHEEL_ROT_ACCEL_RAD, obj.MAX_WHEEL_ROT_ACCEL_RAD];
            obj.mpc = MPC(obj.q, lb, ub, lb_u, ub_u);   
        end
        
        function dq = dynamics(obj, t, q, u)
            % Store parameters
            th = q(3,:);
            phi_dot_L = q(4,:);
            phi_dot_R = q(5,:);
            phi_ddot_L = u(1,:);
            phi_ddot_R = u(2,:);
            % Define our derivatives
            dq = zeros(size(q)); % Pre-allocating
            dq(1,:) = obj.r/2*(phi_dot_L + phi_dot_R).*cos(th);
            dq(2,:) = obj.r/2*(phi_dot_L + phi_dot_R).*sin(th);
            dq(3,:) = obj.r/(obj.b*2)*(phi_dot_L - phi_dot_R);
            dq(4,:) = phi_ddot_L;
            dq(5,:) = phi_ddot_R;
        end
        
        function obj = update_state(obj, u, t)
            dynamics = @(t,q) obj.dynamics(t, q, u);
            [tout, yout] = ode45(dynamics, [0 t], obj.q); % Simulation
            x = yout(end,:);
            obj.q = x;
        end

        function car_points = get_points(obj, X, Y, Theta)
            car_points = cell(numel(X),1);
            for i = 1:numel(X)
                x = X(i);
                y = Y(i);
                theta = -Theta(i);
                rot_mat = [
                    cos(theta), sin(theta), x;
                    -sin(theta), cos(theta), y;
                    0, 0, 1
                ];
                car_points{i} = obj.car_dims*rot_mat';
            end

        end
        
        function obj = step(obj, u, dt, Ts)
            % Ensure that the wheel accelerations are within the limits
            u = max(obj.MIN_WHEEL_ROT_ACCEL_RAD, ...
                    min(obj.MAX_WHEEL_ROT_ACCEL_RAD, u));
            % Update_position                       
%             u = [left_wheel_acceleration, right_wheel_acceleration];
            t = 0;
            i = 1;
            while t<Ts
                fin_time = min(t+dt, Ts);
                sim_time = fin_time-t;

                %update state
                if i>size(u,2)
                    disp("u is not long enough");
                    i = i-1;
                end
                input = u(i,:);
                obj = obj.update_state(input', sim_time);

                %get next values
                t = t+sim_time;
                i = i+1;

            end
        end

        % function c = cost_dist(obj, static_obs, dynamic_obs)
        %     c = 0;
        %     for obs =  [static_obs,dynamic_obs]
        %         if obs.shape == "circle"
        %             c = c+obj.cost_dist_single_circle(obj.q(1:2), obs.current_pose(1:2), obs.dims(1));
        %         else
        %             obs_pose = obs.current_pose;
        %             verticies = obs.gen_verticies(obs_pose, obs.dims);
        %             c = c + obj.cost_dist_single(obj.q(1:2),verticies');
        %         end
        %     end
        % end

        function [u, states] = optimize(obj, goal, map, obstacles)
            [u, states] = obj.mpc.optimize(obj, goal, map, obstacles);
        end
        
    end

end


