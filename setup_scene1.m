function [obstacles, map, config_updated] = setup_scene1(configs)
    config_updated = configs;
    %define start and end point
    config_updated("start") = [-10,0,-pi/2];
    config_updated("end") = [10,0];

    %define canvas size
    canvas = [-12 12 -10 10];
    config_updated("canvas") = canvas;

    w_canvas = canvas(2) - canvas(1);
    h_canvas = canvas(4) - canvas(3);

    num_obs = 3;
    map = zeros(2,4,num_obs);
    %center rectangle points
    pts = Obstacle.rectangular([4,0,0], [7,1]);
    map(:,:,1) = pts;
    pts = Obstacle.rectangular([-7,-6,0], [1,12]);
    map(:,:,2) = pts;
    pts = Obstacle.rectangular([-7,6,0], [1,12]);
    map(:,:,3) = pts;

    %setup obstacles
    obs1 = Obstacle("circle1",[-5,0,0],'rectangle',[0.5,0.5],@obs1_pose);
    obs2 = Obstacle("circle2",[-3,0,0],'rectangle',[0.5,0.5],@obs2_pose);
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