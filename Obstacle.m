classdef Obstacle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        initial_pose
        shape
        dims
        
        gen_verticies
        pose
    end
    
    methods
      %init function
      function obj = Obstacle(id, init_pose, shape, dims, pose)
        
        if shape == "rectangle"
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
        
        %pose update function:
        if isa(pose,'function_handle')
            obj.pose = pose;
            
        else
            obj.pose = @obj.static_pose;
        end
      
      end
     
      
      
    end
    methods (Static)

      %generic pose update(no movement)
      function pose = static_pose(obs, t)
          pose = obs.initial_pose;
      end
      
      %verticies generator
      function [pts, radius] = rectangular(pose, dims)
            p = pose;
            h= dims(1);
            w = dims(2);
            R = @(ang) [cos(ang), -sin(ang); sin(ang) cos(ang)];
            
            %define points using offset at 0
            p1 = [-w/2 h/2];
            p2 = [w/2 h/2];
            p3 = [w/2 -h/2];
            p4 = [-w/2 -h/2];
            pts = [p1;p2;p3;p4]';
            pts = R(p(3))*pts; %rotates the points
            
            %offset based on current position
            pts = pts + p(1:2)';
            radius = sqrt((w/2)^2+(h/2)^2);
            
        end
        function [pts, radius] = triangular(pose, dims)
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
            
            radius = sqrt((b/2)^2+(h/2)^2);
            
        end
        function [pts, radius] = circular(pose, dims)
            p = pose;
            [r] = dims;
            
            pts = [];
            radius = r;
        end
    end
end