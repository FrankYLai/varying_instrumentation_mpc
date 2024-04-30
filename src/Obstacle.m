classdef Obstacle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        initial_pose
        shape
        dims
        t
        current_pose
    
        
        gen_verticies
        pose
        tracking
    end
    
    methods
      %init function
      function obj = Obstacle(id, init_pose, shape, dims, pose, tracking)
        obj.t = 0;
        
        if shape == "rectangle"
            %redefine so that height length>width
            if (dims(1)<dims(2))
                t = dims(1);
                dims(1) = dims(2);
                dims(2) = t;
                init_pose(3) = init_pose(3)+pi/2;
            end
            
            obj.gen_verticies = @obj.rectangular;
        elseif shape == "triangle"
            obj.gen_verticies = @obj.triangular;
        elseif shape == "circle"
            obj.gen_verticies = @obj.circular;
        else
            disp(["invalid shape", shape])
        end
        
        obj.id = id;
        obj.dims = dims;
        obj.initial_pose = init_pose;
        obj.shape =  shape;
        obj.current_pose = init_pose;

        
        %pose update function:
        if isa(pose,'function_handle')
            obj.pose = pose;
        else
            obj.pose = @obj.static_pose;
        end

        %pose tracking function:
        obj.tracking = tracking;
      end
     
      %run to update the position of an obstacle
      function obj = updatePose(obj, t)
          obj.t = t;
          pose_update = obj.pose(obj, obj.t);
          obj.current_pose = pose_update;
      end
      
      function pose = predict_pose(obj, dt)
        if obj.tracking == "full"
            pose = obj.predict_pose_full(obj, dt);
        elseif obj.tracking == "simple"
            pose = obj.predict_pose_simple(obj, dt);
        else
            pose = obj.predict_pose_none(obj, dt);
        end
      end
      
    end
    methods (Static)

      %generic pose update(no movement)
      function pose = static_pose(obs, t)
          pose = obs.initial_pose;
      end
      
      %pose prediction
      function pose = predict_pose_full(obj,dt)
            pose = obj.pose(obj, obj.t+dt);
      end
      
      function pose = predict_pose_simple(obj,dt)
        h = 0.1;
        pose_old = obj.pose(obj, obj.t-h);
        diff = obj.current_pose - pose_old;
        dx = diff(1)/h;
        dy = diff(2)/h;
        dtheta = diff(3)/h;

        pose = [dx*dt + obj.current_pose(1), ...
                dy*dt + obj.current_pose(2), ...
                dtheta*dt + obj.current_pose(3)];
      end

      function pose = predict_pose_none(obj, dt)
        pose = obj.pose(obj, obj.t);
      end
      
      %verticies generator
      function pts = rectangular(pose, dims)
            p = pose;
            l= dims(1);
            h = dims(2);
            R = @(ang) [cos(ang), -sin(ang); sin(ang) cos(ang)];
            
            %define points using offset at 0
            p1 = [-l/2 h/2];
            p2 = [l/2 h/2];
            p3 = [l/2 -h/2];
            p4 = [-l/2 -h/2];
            pts = [p1;p2;p3;p4]';
            pts = R(p(3))*pts; %rotates the points
            
            %offset based on current position
            pts = pts + p(1:2)';
            
        end
        function pts = triangular(pose, dims)
            p = pose;
            b= dims(1);
            h = dims(2);
            R = @(ang) [cos(ang), -sin(ang); sin(ang) cos(ang)];
            
            %define points using offset at 0
            p1 = [0, h/2];
            p2 = [-b/2, -h/2];
            p3 = [b/2, -h/2];
            pts = [p1;p2;p3]';
            pts = R(p(3))*pts; %rotate
            
            %offset based on current position
            pts = pts + p(1:2)';
            
        end
        function pts = circular(pose, dims)
            p = pose;
            r = dims(1);
            
            %larger the circle more points to use to draw it
            divisions = r*20;
            th = 0:pi/divisions:2*pi;
            xunit = r * cos(th);
            yunit = r * sin(th);
            
            pts = [xunit;yunit] + p(1:2)';
        end
    end
end