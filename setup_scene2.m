function [obstacles, map, config_updated] = setup_scene2(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [0,0,0];
    config_updated("end") = [8,10];

    %define canvas size
    canvas = [-14 14 -12 12];
    config_updated("canvas") = canvas;
    

    map = [];



    %setup obstacles
%     obs1 = Obstacle(4,[0,0,0],'circle',[3, 3],@obs1_pose, configs("tracking"));
    obs1 = Obstacle(4,[0,0,0],'circle',[1, 1],@obs1_pose, configs("tracking"));
    obs2 = Obstacle(4,[0,0,0],'circle',[1, 1],@obs2_pose, configs("tracking"));
    obs3 = Obstacle(4,[0,0,0],'circle',[1, 1],@obs3_pose, configs("tracking"));
    obs4 = Obstacle(4,[0,0,0],'circle',[1, 1],@obs4_pose, configs("tracking"));
    obstacles = [obs1, obs2, obs3, obs4];
%     obstacles = []

end


function pose = obs1_pose(obj, t)
    diff = [4*sin(t/2), 4*cos(t/2), 0];
    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [4*sin(t/2+pi), 4*cos(t/2+pi), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs3_pose(obj, t)
    diff = [6*sin(t/2+pi/2), 6*cos(t/2+pi/2), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs4_pose(obj, t)
    diff = [6*sin(t/2+pi*3/2), 6*cos(t/2+pi*3/2), 0];

    pose = diff + obj.initial_pose;
end