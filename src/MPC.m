classdef MPC
    properties
        Th = 2;   % Prediction horizon
        optim_options = optimoptions('fmincon');
        N_states = 21;    % Number of states
        N_vals = 5;
        N_inputs = 2;
        
        dt;
        w;
        LB;
        UB;
        scene_snapshots;
    end
    
    methods
        function obj = MPC(lb, ub, lb_u, ub_u)
            % Initialize the MPC controller
            obj.optim_options.Display = 'final';
            obj.optim_options.MaxFunctionEvaluations = 5e4;
            obj.optim_options.MaxIterations = 5e4;

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

            %save a copy of obstacles in each future state
            obj.scene_snapshots = containers.Map();
            % saved in [x1_obs1, x2_obs1, ..., xn_obs1,]
            all_obs = [map, obstacles];
            t = 0;
            for i = 1:obj.N_states
                obstacle_verticies = Verticies.empty;
                for obs_idx = 1:size(all_obs,2)
                    pose = all_obs(obs_idx).predict_pose(t);
                    verticies = all_obs(obs_idx).gen_verticies(pose, all_obs(obs_idx).dims);
%                     verticies
                    circle = all_obs(obs_idx).shape=="circle";
                    o = Verticies(circle, all_obs(obs_idx).dims(1), pose);
                    o.x = verticies(1,:);
                    o.y = verticies(2,:);
                    obstacle_verticies(1,obs_idx) = o;
                end
%                 obstacle_verticies
                obj.scene_snapshots(num2str(i)) = obstacle_verticies;
                t = t + obj.dt;
                
            end
            
            
            % Define the cost function
            cost_func_handle = @(w) obj.costfunc(w, goal);

            % Define the constraints
            nonlcon = @(w) obj.nonlcon(w, car, map, obstacles);

%             for i = 1:obj.N_states
%                 obstacle_verticies = scene_snapshots(num2str(i));
%                 figure(i)
%                 hold on;
%                 i
%                 for j = 1:size(obstacle_verticies,2)
%                     o = obstacle_verticies(j);
%                     o.x, o.y
%                     fill(o.x, o.y, 'k');
%                 end
%             end

            % Solve the optimization problem
            [obj.w, FVAL] = fmincon(cost_func_handle, obj.w, [], [], [], [], obj.LB, obj.UB, nonlcon, obj.optim_options);
            FVAL
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
                                                                o.r);
                    else
                        cost = cost + obj.cost_dist_single(states(i,1:2), [o.x', o.y']);
                    end
                end
            end
            cost = cost/obj.N_states;
%             disp("cost_obstacle "+num2str(cost))
        end
        function cost = costfunc(obj, w, goal)
            % Calculate the total cost of the MPC controller
            vals = reshape(w, obj.N_states, obj.N_vals+obj.N_inputs);
            inputs = vals(:,1:2);
            states = vals(:,3:end);
            cost_dist =  obj.cost_dist(states, goal);
            
            cost_obstacle = obj.cost_obstacle(states);
%             cost_dist
%             cost_obstacle
            cost = (0.5+cost_obstacle)*cost_dist;
%             cost =  cost_dist + cost_obstacle;
        end

        % function [cross_section, collision_objs] = checkCollision(obj, car, static_obs, dynamic_obs)
        %     obs_arry = [static_obs,dynamic_obs];

        %     pts_ego = car.get_points();
        %     poly_ego = polyshape(pts_ego(:,1), pts_ego(:,2));
        %     collision_objs = [];
        %     cross_section = 0;
        %     for obs = obs_arry
        %         pts_obs = obs.gen_verticies(obs.current_pose, obs.dims);
        %         poly_obs = polyshape(pts_obs(1,:), pts_obs(2,:));
        %         polyout = intersect(poly_ego,poly_obs);
        %         cross_section = cross_section + area(polyout);
        %         if area(polyout)>0
        %             collision_objs =  polyout;
        %         end
        %     end
        % end

        function [C, Ceq] = nonlcon(obj, w, car, static_obs, dynamic_obs)
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
            
            Ceq = [defect(:); start_cond(:)];
            C = [];


            %     car.current_pose = states(:,i);
            %     car.current_vel = states(:,i+obj.N_states);
            %     car.current_input = inputs(:,i);
            %     [cross_section, collision_objs] = checkCollision(obj, car, static_obs, dynamic_obs);
            %     C = [C;cross_section];
            % end
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

        function c = cost_dist_single_circle(pt, center, r)
            safety = 0.5;
            v = pt-center;
            d = max(sqrt(sum(v.^2))-r-safety,0);
            if d == 0
                pt
                center
            end
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



