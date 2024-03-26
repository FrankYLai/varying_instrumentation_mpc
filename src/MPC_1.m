classdef MPC_1
    properties
        horizon    % Prediction horizon
        R          % Input cost matrix
        Rd         % Input difference cost matrix
        Q          % State cost matrix
        Qf         % Final state cost matrix
    end
    
    methods
        function obj = MPC(horizon)
            obj.horizon = horizon;  % Set the prediction horizon
            obj.R = diag([0.01, 0.01]);    % Input cost matrix, penalizing large control actions
            obj.Rd = diag([0.01, 1.0]);    % Input difference cost matrix, penalizing rapid changes in control actions
            obj.Q = diag([1.0, 1.0]);      % State cost matrix, penalizing deviations from the desired state
            obj.Qf = obj.Q;                % Final state cost matrix, same as state cost matrix
        end
        
        function cost_val = cost(obj, u_k, car, goal_x)
            goal_x = goal_x';  % Transpose the goal state vector
            controller_car = copy(car);  % Create a copy of the car object
            u_k = reshape(u_k, 2, obj.horizon);  % Reshape the control sequence into a matrix
            z_k = zeros(2, obj.horizon+1);  % Initialize array to store state trajectory
            desired_state = goal_x;  % Set the desired state
            cost_val = 0.0;  % Initialize cost value
            
            for i = 1:obj.horizon
                controller_car.set_robot_velocity(u_k(1,i), u_k(2,i));  % Set control input on the car object
                controller_car.update(DELTA_T);  % Update the car's state based on the control input
                [x, ~] = controller_car.get_state();  % Get the current state of the car
                z_k(:,i) = [x(1,1); x(2,1)];  % Store the current state in the trajectory array
                cost_val = cost_val + sum(sum(obj.R * (u_k(:,i).^2)));  % Add control cost to the total cost
                cost_val = cost_val + sum(sum(obj.Q * ((desired_state - z_k(:,i)).^2)));  % Add state cost to the total cost
                if i < obj.horizon
                    cost_val = cost_val + sum(sum(obj.Rd * ((u_k(:,i+1) - u_k(:,i)).^2)));  % Add input difference cost to the total cost
                end
            end
        end
        
        function [u_optimized_1, u_optimized_2] = optimize(obj, car, goal_x)
            % Define bounds for control inputs over the prediction horizon
            bnd = repmat([MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY, MIN_WHEEL_ROT_SPEED_RAD, MAX_WHEEL_ROT_SPEED_RAD], 1, obj.horizon);
            options = optimoptions('fmincon','Display','iter');  % Define optimization options
            initial_guess = zeros(2 * obj.horizon, 1);  % Initialize the initial guess for control inputs
            % Perform constrained optimization to minimize the cost
            result = fmincon(@(u) obj.cost(u, car, goal_x), initial_guess, [], [], [], [], [], [], [], options);
            u_optimized_1 = result(1);  % Extract optimized control input 1
            u_optimized_2 = result(2);  % Extract optimized control input 2
        end
    end
end
