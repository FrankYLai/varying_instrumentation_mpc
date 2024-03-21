function animate(configs, car, dynamic, map, path, t)
    clf
    hold on;

    %draw start and end goals
    end_pos = configs("end");
    start_pos = configs("start");
    
    plot(start_pos(1),start_pos(2),'bo');
    plot(end_pos(1),end_pos(2),'g*');

    for obs = map %draw static obstacles
        obs_pose = obs.current_pose;
        verticies = obs.gen_verticies(obs_pose, obs.dims);
        fill(verticies(1,:), verticies(2,:), 'k');
    end
    
    for obs = dynamic %draw dynamic obstacles
        obs_pose = obs.current_pose;
        verticies = obs.gen_verticies(obs_pose, obs.dims);
        fill(verticies(1,:), verticies(2,:), 'r');
    end

    %draw car:
    points = car.get_points();
    fill(points(:,1), points(:,2), 'g');
    
    
    axis(configs("canvas"));
    drawnow;
    hold off;
end