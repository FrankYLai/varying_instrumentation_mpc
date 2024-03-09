classdef Obstacle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        id
        inital_pose
        dims
        currnet_pose
        
        gen_verticies
        pose
    end
    
    methods
      %init function
      function obj = Obstacle(id, init_pose, shape, dims, pose)
        if not(all(init_pose>=0))
            disp(["invalid pose", num2str(pose)])
            return
        end
        
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
        obj.inital_pose = init_pose;
        obj.pose = pose;
        
        obj.currnet_pose = init_pose;
      end
     
      
      
    end
    methods (Static)
      
      %verticies generator
      function [pts, radius] = rectangular(pose, dims)
            p = pose;
            h= dims(1);
            w = dims(2);
            R = @(ang) [cos(ang), -sin(ang); sin(ang) cos(ang)];
            
            %define points using offset
            p1 = p(1:2) + [-w/2 h/2];
            p2 = p(1:2) + [w/2 h/2];
            p3 = p(1:2) + [w/2 -h/2];
            p4 = p(1:2) + [-w/2 -h/2];
            pts = [p1;p2;p3;p4]';
            pts = R(p(3))*pts; %rotates the points
            radius = sqrt((w/2)^2+(h/2)^2);
            
        end
        function [pts, radius] = triangular(pose, dims)
            disp(dims)
            p = pose;
            b= dims(1);
            h = dims(2);
            R = @(ang) [cos(ang), -sin(ang); sin(ang) cos(ang)];
            
            p1 = p(1:2) + [0, h/2];
            p2 = p(1:2) + [-b/2, -h/2];
            p3 = p(1:2) + [b/2, -h/2];
            pts = [p1;p2;p3]';
            pts = R(p(3))*pts;
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