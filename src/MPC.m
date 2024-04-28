classdef MPC
    properties
        Th = 1;   % Prediction horizon
        optim_options = optimoptions('fmincon');
        N_states = 11;    % Number of states
        N_vals = 5;
        N_inputs = 2;

        dt;
        w;
        LB;
        UB;
    end
    
    methods
        function obj = MPC(lb, ub, lb_u, ub_u)
            % Initialize the MPC controller
            obj.optim_options.Display = 'off';
            obj.optim_options.MaxFunctionEvaluations = 1e4;
            obj.optim_options.MaxIterations = 1e4;

            % Initialize the control input and state vector
            obj.dt = obj.Th/(obj.N_states-1);
            obj.w = zeros(obj.N_states, obj.N_vals+obj.N_inputs);
            obj.w = obj.w(:);

            %initialize the lower and upper bounds
            LB_U = [ones(obj.N_states,1)*lb_u(1);ones(obj.N_states,1)*lb_u(2)];
            UB_U = [ones(obj.N_states,1)*ub_u(1);ones(obj.N_states,1)*ub_u(2)];
            LB_S = [ones(obj.N_states,1)*lb(1);ones(obj.N_states,1)*lb(2);...
                    ones(obj.N_states,1)*lb(3);ones(obj.N_states,1)*lb(4);ones(obj.N_states,1)*lb(5)];
            UB_S = [ones(obj.N_states,1)*ub(1);ones(obj.N_states,1)*ub(2);...
                    ones(obj.N_states,1)*ub(3);ones(obj.N_states,1)*ub(4);ones(obj.N_states,1)*ub(5)];
            obj.LB = [LB_U;LB_S];
            obj.UB = [UB_U;UB_S];

        end
        
        function [u, x]= optimize(obj, car, goal, map, obstacles)

            % Define the cost function
            cost_func_handle = @(w) obj.costfunc(car, w, goal, map, obstacles);

            % Define the constraints
            nonlcon = @(w) obj.nonlcon(w, car, map, obstacles);
            % Solve the optimization problem
            obj.w = fmincon(cost_func_handle, obj.w, [], [], [], [], obj.LB, obj.UB, nonlcon, obj.optim_options);
            % Return the optimal control input and the optimal trajectory

            reshape_w = reshape(obj.w, obj.N_states, obj.N_vals+obj.N_inputs);
            u = reshape_w(:,1:2);
            x = reshape_w(:,3:end);

        end

        function cost = cost_dist(obj, states, goal)
            % Calculate the cost of the distance between the ego car and the goal
            pos = states(:,1:2);
            cost = sum((pos-goal).^2,'all');
        end
        function cost = cost_obstacle(obj, states, car, static_obs, dynamic_obs)
            cost = 0;
            % calculate the cost of the distance between the ego car and the obstacles
        end
        function cost = costfunc(obj, w, car, goal, static_obs, dynamic_obs)
            % Calculate the total cost of the MPC controller
            vals = w.reshape(obj.N_states, obj.N_vals+obj.N_inputs);
            inputs = vals(:,1:2);
            states = vals(:,3:end);
            cost =  cost_dist(w, states, goal) + cost_obstacle(states, car, static_obs, dynamic_obs);
        end

        function [cross_section, collision_objs] = checkCollision(obj, car, static_obs, dynamic_obs)
            obs_arry = [static_obs,dynamic_obs];

            pts_ego = car.get_points();
            poly_ego = polyshape(pts_ego(:,1), pts_ego(:,2));
            collision_objs = [];
            cross_section = 0;
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

        function [C, Ceq] = nonlcon(obj, w, car, static_obs, dynamic_obs)
            % Define the nonlinear constraints
            vals = w.reshape(obj.N_states, obj.N_vals+obj.N_inputs);
            u = vals(:,1:2);
            q = vals(:,3:end);
            
            %defect constraints
            dq = car.dynamics(obj, [], q(1:end-1,:), u(1:end-1,:));
            defect = q(2,end) - q(1,end-1) - dq*obj.dt;
            start_cond = q(1,:) - car.q;
            Ceq = [defect; start_cond];

            C = [];


            %collision detection
            % for i = 1:obj.N_vals
            %     car.current_pose = states(:,i);
            %     car.current_vel = states(:,i+obj.N_states);
            %     car.current_input = inputs(:,i);
            %     [cross_section, collision_objs] = checkCollision(obj, car, static_obs, dynamic_obs);
            %     C = [C;cross_section];
            % end
        end
        
    end
end
