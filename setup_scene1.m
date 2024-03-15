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
    [pts, radius] = Obstacle.rectangular([0,0,0], [5,1]);
    map(:,:,1) = pts;
    [pts, radius] = Obstacle.rectangular([-4,-4,0], [5,1]);
    map(:,:,2) = pts;
    [pts, radius] = Obstacle.rectangular([-4,4,0], [5,1]);
    map(:,:,3) = pts;

    %setup obstacles
    obs1 = Obstacle("rectangle_example",[-3,-2,pi/3],'rectangle',[0.4,0.2],@obs1_pose);
    obs2 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],NaN);
    obs3 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],NaN);


    obstacles = [obs1, obs2, obs3];
end


function pose = obs1_pose(obj, t)
    diff = [0.5*sin(t/2), 2*cos(t), 0];
    x = 2*sin(t/2) + obj.initial_pose(1);
    y = obj.initial_pose(2);
    theta = obj.initial_pose(3);

    pose = diff + obj.initial_pose;
end