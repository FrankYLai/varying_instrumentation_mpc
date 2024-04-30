function [obstacles, map, config_updated] = setup_scene3(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [-10,0,0];
    config_updated("end") = [10,0];

    %define canvas size
    canvas = [-12 12 -10 10];
    config_updated("canvas") = canvas;
    
    stat_obs3 = Obstacle(1,[0,107,0],'circle',[100,100],[], configs("tracking"));
    stat_obs1 = Obstacle(1,[0,-107,0],'circle',[100,100],[], configs("tracking"));
%     stat_obs1 = Obstacle(1,[4,0,pi/2],'rectangle',[7,1],[], configs("tracking"));
    
    %add static obstacles to list
    map = [stat_obs3, stat_obs1];
    map = [];



    %setup obstacles
%     obs1 = Obstacle(4,[0,0,0],'circle',[3, 3],@obs1_pose, configs("tracking"));
    obs1 = Obstacle(4,[0,0,0],'circle',[3, 3],@obs1_pose, configs("tracking"));

    obstacles = [obs1];
    

end


function pose = obs1_pose(obj, t)
    diff = [0, 3*sin(t), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [0, -5.2*sin(t/2), 0];

    pose = diff + obj.initial_pose;
end