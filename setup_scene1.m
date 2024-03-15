function [obstacles, map, config_updated] = setup_scene1(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [-10,0,-pi/2];
    config_updated("end") = [10,0];

    %define canvas size
    canvas = [-12 12 -10 10];
    config_updated("canvas") = canvas;
    
    %scene def to detect collisions
    config_updated("obsList") = dynamicCapsuleList;

%     w_canvas = canvas(2) - canvas(1);
%     h_canvas = canvas(4) - canvas(3);

    stat_obs1 = Obstacle(1,[4,0,0],'rectangle',[7,1],[]);
    stat_obs2 = Obstacle(2,[-7,-6,0],'rectangle',[1,12],[]);
    stat_obs3 = Obstacle(3,[-7,6,0],'rectangle',[1,12],[]);

    map = [stat_obs1, stat_obs2, stat_obs3];


    %setup obstacles
    obs1 = Obstacle("circle1",[-5,0,0],'circle',[0.5,0.5],@obs1_pose);
    obs2 = Obstacle("circle2",[-3,0,0],'circle',[0.5,0.5],@obs2_pose);
%     obs3 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],NaN);
%     obs4 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],NaN);


    obstacles = [obs1, obs2];
end


function pose = obs1_pose(obj, t)
    diff = [0, 4*sin(t), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [0, -4*sin(t/2), 0];

    pose = diff + obj.initial_pose;
end