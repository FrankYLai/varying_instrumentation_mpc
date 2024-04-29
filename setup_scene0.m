function [obstacles, map, config_updated] = setup_scene0(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [-10,0,0];
    config_updated("end") = [10,2];

    %define canvas size
    canvas = [-12 12 -10 10];
    config_updated("canvas") = canvas;
    
%     stat_obs3 = Obstacle(1,[-7,6,0],'rectangle',[12,1],[], configs("tracking"));
%     stat_obs1 = Obstacle(1,[4,0,pi/2],'rectangle',[7,1],[], configs("tracking"));
    
    %add static obstacles to list
    map = [];


    %setup obstacles
    obs1 = Obstacle(4,[-2,-0.5,0],'circle',[0.25],@obs1_pose, configs("tracking"));
    % obs2 = Obstacle(5,[-1,0,0],'circle',[0.25],@obs2_pose);
%     obs3 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],NaN);
%     obs4 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],NaN);
    obstacles = [obs1];
    

end


function pose = obs1_pose(obj, t)
    diff = [0, 3*cos(3*t), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [0, -5.2*sin(t/2), 0];

    pose = diff + obj.initial_pose;
end