function [obstacles, map, config_updated] = setup_scene2(configs)
    config_updated = configs;
    %define canvas size
    canvas = [-12 12 -8 8];
    config_updated("canvas") = canvas;

    %define start and end point
    config_updated("start") = [-10,0,0];
    config_updated("end") = [0,5];
    
    
    %static Obstacle
    stat_obs1 = Obstacle(1,[-7,5,0],'rectangle',[10,6],[]); %-12 -2, 2, 8
    stat_obs2 = Obstacle(2,[-7,-5,0],'rectangle',[10,6],[]);
    stat_obs3 = Obstacle(3,[7,5,0],'rectangle',[10,6],[]);
    stat_obs4 = Obstacle(4,[7,-5,0],'rectangle',[10,6],[]);
    
    %add static obstacles to list
    map = [stat_obs1, stat_obs2, stat_obs3, stat_obs4];


    %setup dynamic obstacles
    obs1 = Obstacle(5,[8,1,0],'rectangle',[1.5, 0.75],@obs1_pose);
    obs2 = Obstacle(6,[1,-7,0],'rectangle',[0.75, 1.5],@obs2_pose);
%     obs3 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],NaN);
%     obs4 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],NaN);
    obstacles = [obs1, obs2];
    

end


function pose = obs1_pose(obj, t)
    diff = [-1.3*t, 0, 0];
    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [0, t, 0];

    pose = diff + obj.initial_pose;
end