function [obstacles, map, config_updated] = setup_scene1(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [-12,-10,0];
    config_updated("end") = [8,10];

    %define canvas size
    canvas = [-14 14 -12 12];
    config_updated("canvas") = canvas;
    

    stat_obs3 = Obstacle(1,[0,107,0],'circle',[100,100],[], configs("tracking"));
    stat_obs1 = Obstacle(1,[0,-107,0],'circle',[100,100],[], configs("tracking"));
%     stat_obs1 = Obstacle(1,[4,0,pi/2],'rectangle',[7,1],[], configs("tracking"));
    
    %add static obstacles to list
    map = [stat_obs3, stat_obs1];
    map = [];



    %setup obstacles
%     obs1 = Obstacle(4,[0,0,0],'circle',[3, 3],@obs1_pose, configs("tracking"));
    obs1 = Obstacle(4,[-7,-3,0],'circle',[2, 2],@obs1_pose, configs("tracking"));
    obs2 = Obstacle(4,[7,5,0],'circle',[2, 2],@obs2_pose, configs("tracking"));
    obstacles = [obs1, obs2];
%     obstacles = []

end


function pose = obs1_pose(obj, t)
    diff = [2*t, 0, 0];


    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [-2*t, 0, 0];

    pose = diff + obj.initial_pose;
end