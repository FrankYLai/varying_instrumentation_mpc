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
    

    stat_obs1 = Obstacle(1,[4,0,pi/2],'rectangle',[7,1],[]);
    stat_obs2 = Obstacle(2,[-7,-6,0],'rectangle',[12,1],[]);
    stat_obs3 = Obstacle(3,[-7,6,0],'rectangle',[12,1],[]);
    
    %add static obstacles to list
    map = [stat_obs1, stat_obs2, stat_obs3];
    for wall = map
        addObstacle(config_updated("obsList"),wall.obsCapsule);
    end


    %setup obstacles
    obs1 = Obstacle(4,[-8,0,0],'circle',[0.25],@obs1_pose);
    obs2 = Obstacle(5,[-1,0,0],'circle',[0.25],@obs2_pose);
%     obs3 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],NaN);
%     obs4 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],NaN);
    obstacles = [obs1, obs2];
    
    %add dynamic obstacles to list
    for obs = obstacles
        addObstacle(config_updated("obsList"),obs.obsCapsule);
    end

end


function pose = obs1_pose(obj, t)
    diff = [0, 5.2*sin(t), 0];

    pose = diff + obj.initial_pose;
end

function pose = obs2_pose(obj, t)
    diff = [0, -5.2*sin(t/2), 0];

    pose = diff + obj.initial_pose;
end