function animate(configs, car, dynamic, map, path, t)


    clf
    hold on;
    % draw map
    map_shape = size(map);
    for i=(1:map_shape(3))
        fill(map(1,:,i), map(2,:,i), 'k');
    end
    
    for obs = dynamic %draw dynamic obstacles
        obs_pose = obs.pose(obs, t);
        verticies = obs.gen_verticies(obs_pose, obs.dims);
        fill(verticies(1,:), verticies(2,:), 'r');
    end
    
    %draw start and end goals
    end_pos = configs("end");
    start_pos = configs("start");
    
    plot(start_pos(1),start_pos(2),'bo')
    plot(end_pos(1),end_pos(2),'g*')
    
    
    axis(configs("canvas"))
    drawnow
    hold off;
end