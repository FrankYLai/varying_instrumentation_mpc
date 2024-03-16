classdef Obstacle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        initial_pose
        shape
        dims
        obsCapsule
        current_pose
        
        gen_verticies
        pose
    end
    
    methods
      %init function
      function obj = Obstacle(id, init_pose, shape, dims, pose)
        geom = [];
        
        if shape == "rectangle"
            %redefine so that height length>width
            if (dims(1)<dims(2))
                t = dims(1);
                dims(1) = dims(2);
                dims(2) = t;
                init_pose(3) = init_pose(3)+pi/2;
            end
            transform = eye(3);
            transform(1,3) = -dims(1)/2;
            geom = struct("Length",dims(1),"Radius",dims(2)/2,"FixedTransform",transform);
            
            obj.gen_verticies = @obj.rectangular;
        elseif shape == "triangle"
            obj.gen_verticies = @obj.triangular;
        elseif shape == "circle"
            geom = struct("Length",0.01,"Radius",dims(1),"FixedTransform",eye(3));
            obj.gen_verticies = @obj.circular;
        else
            disp(["invalid shape", shape])
        end
        
        obj.id = id;
        obj.dims = dims;
        obj.initial_pose = init_pose;
        obj.shape =  shape;
        obj.current_pose = init_pose;

        %maintain obs capsul to check for collisions
        obj.obsCapsule = struct('ID',id,'States',init_pose,'Geometry',geom);
        
        %pose update function:
        if isa(pose,'function_handle')
            obj.pose = pose;
        else
            obj.pose = @obj.static_pose;
        end
      
      end
     
      %run to update the position of an obstacle
      function obj = updatePose(obj, t, configs)
          pose_update = obj.pose(obj, t);
          obj.obsCapsule.States = pose_update;
          updateObstaclePose(configs("obsList"), obj.id, obj.obsCapsule);
          obj.current_pose = pose_update;
      end
      
    end
    methods (Static)

      %generic pose update(no movement)
      function pose = static_pose(obs, t)
          pose = obs.initial_pose;
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