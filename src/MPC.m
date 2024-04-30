classdef MPC
    properties
        Th = 2.5;   % Prediction horizon
        optim_options = optimoptions('fmincon');
        N_states = 26;    % Number of states
        N_vals = 5;
        N_inputs = 2;
        safety = 0.5;
        edge = 2.5;
        
        dt;
        w;
        LB;
        UB;
        N_obs;
    end
    
    methods
        function obj = MPC(initial_pos, lb, ub, lb_u, ub_u)
            % Initialize the MPC controller
            obj.optim_options.Display = 'final';
            obj.optim_options.MaxFunctionEvaluations = 1e5;
            obj.optim_options.MaxIterations = 1e5;

            % Initialize the control input and state vector
            obj.dt = obj.Th/(obj.N_states-1);
            obj.w = obj.set_w(initial_pos);

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

            %save a copy of obstacles in each future state
            % saved in [x1_obs1, x2_obs1, ..., xn_obs1,]
            all_obs = [map, obstacles];
            obj.N_obs = size(all_obs,2);

            R = zeros(obj.N_obs*obj.N_states,1);
            centers = zeros(obj.N_obs*obj.N_states,2);
            t = 0;
            for i = 1:obj.N_states
                for obs_idx = 1:obj.N_obs
                    %compute the index
                    idx = (i-1)*obj.N_obs + obs_idx;
                    %compute the pose
                    pose = all_obs(obs_idx).predict_pose(t);
                    centers(idx,:) = pose(1:2);
                    R(idx,1) = all_obs(obs_idx).dims(1);
                end
                t = t + obj.dt;
                
            end
            
            
            % Define the cost function
            cost_func_handle = @(w) obj.costfunc(w, goal, centers, R);

            % Define the constraints
            nonlcon = @(w) obj.nonlcon(w, car, centers, R);
            
            temp_w = obj.set_w(car.q);
            % Solve the optimization problem
            [w, FVAL, flag] = fmincon(cost_func_handle, temp_w, [], [], [], [], obj.LB, obj.UB, nonlcon, obj.optim_options);
%             if flag == 0
%                 disp("Optimization did not converge, retrying with different initial conditions")
%                 temp_w = obj.set_w(car.q);
%                 [w, FVAL, flag] = fmincon(cost_func_handle, temp_w, [], [], [], [], obj.LB, obj.UB, nonlcon, obj.optim_options);
%             end
            obj.w = w;
            % Return the optimal control input and the optimal trajectory

            reshape_w = reshape(obj.w, obj.N_states, obj.N_vals+obj.N_inputs);
            u = reshape_w(:,1:2); %[L, R]
            x = reshape_w(:,3:end);

        end

        function cost = cost_dist(obj, states, goal)
            % Calculate the cost of the distance between the ego car and the goal
            pos = states(:,1:2);
            cost = sum((pos-goal).^2,'all');
%             disp("cost_dist "+num2str(cost))
        end
        function cost = cost_obstacle(obj, states)
            % calculate the cost of the distance between the ego car and the obstacles
            cost = 0;
            for i = 1:obj.N_states
                obstacle_verticies = obj.scene_snapshots(num2str(i));
                for j = 1:size(obstacle_verticies,2)
                    o = obstacle_verticies(j);
                    if o.circle
                        cost = cost+obj.cost_dist_single_circle(states(i,1:2),...
                                                                o.pose(1:2),...
                                                                o.r, obj.safety);
                    else
                        disp("not implemented")
                    end
                end
            end
            cost = cost/obj.N_states;
%             disp("cost_obstacle "+num2str(cost))
        end
        function cost = costfunc(obj, w, goal, centers, R)
            % Calculate the total cost of the MPC controller
            vals = reshape(w, obj.N_states, obj.N_vals+obj.N_inputs);
            inputs = vals(:,1:2);
            
            %distance cost
            pos = vals(:,3:4);
            cost = sum((pos-goal).^2,'all');
            
            %obstacle cost
            pos_expanded = repelem(pos, obj.N_obs, 1);
            dists = sqrt(sum((pos_expanded(:,1:2) - centers).^2,2));
            obstacle_cost = (R+obj.safety+obj.edge) - dists;
            obstacle_cost = max(obstacle_cost,zeros(size(obstacle_cost,1),1)); %Relu
            cost = cost+sum(obstacle_cost)*15;
            

            % cost = cost_dist;
%             cost =  cost_dist + cost_obstacle;
        end

        function [C, Ceq] = nonlcon(obj, w, car, centers, R)
            % Define the nonlinear constraints
            vals = reshape(w, obj.N_states, obj.N_vals+obj.N_inputs);
            u = vals(:,1:2);
            q = vals(:,3:end);
            
            %defect constraints
            dq = car.dynamics([], q(1:end-1,:)', u(1:end-1,:)');
            dq = dq';
            defect = q(2:end,:) - q(1:end-1,:) - dq*obj.dt;
            start_cond = q(1,:) - car.q;
            
            %collision detection
            % intersect = checkCollision(q, static_obs, dynamic_obs, 0:obj.dt:obj.Th);
            % for i = 1:obj.N_states
            %     obstacle_verticies = obj.scene_snapshots(num2str(i));
            %     for j = 1:size(obstacle_verticies,2)
            %         o = obstacle_verticies(j);

            %     end
            % end
            
            Ceq = [defect(:); start_cond(:)];
            
            
%             C = [];
            %compute inequality constraints
            w_expanded = repelem(q, obj.N_obs, 1);
            dists_sq = sum((w_expanded(:,1:2) - centers).^2,2);
            C = (R+obj.safety).^2 - dists_sq;
            


        end

        function w = set_w(obj, pos)
%             w = zeros(obj.N_states, obj.N_vals+obj.N_inputs);
%             w(:,3) = ones(obj.N_states,1)*pos(1);
%             w(:,4) = ones(obj.N_states,1)*pos(2);
%             w(:,5) = ones(obj.N_states,1)*pos(3);
%             w = w(:);
            w = repelem([[0;0];pos'],obj.N_states,1);
        end
        
    end

    methods(Static)

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
            c = 1/(mindist + 0.001);
        
        end

        function c = cost_dist_single_circle(pt, center, r, safety)
            v = pt-center;
            d = max(sqrt(sum(v.^2))-r - safety,0);

            c = 1/(d + 0.001);

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
    dist = sum((nearest-pnt_vec).^2);
%     dist = sqrt(sum((nearest-pnt_vec).^2));% # 0.985
end



