classdef Verticies
    %VERTICIES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x;
        y;
        circle;
        r;
        pose;
    end
    
    methods
        function obj = Verticies(circle,r,pose)
            obj.circle = circle;
            obj.r = r;
            obj.pose = pose;
        end
    end
end

