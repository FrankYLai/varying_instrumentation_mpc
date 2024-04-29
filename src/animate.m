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
    points = car.get_points(car.q(1), car.q(2), car.q(3));
    points = points{1};
    fill(points(:,1), points(:,2), 'g');
    center = [car.q(1), car.q(2)];
    % plot circle at center
    radius = 0.1; 
    theta = linspace(0, 2*pi, 25);
    x = center(1) + radius*cos(theta);
    y = center(2) + radius*sin(theta);
    fill(x, y, 'r');
    
    %plot path
    x = path(:,1);
    y = path(:,2);
    plot(x, y, '-o');
    
    
    axis(configs("canvas"));
    drawnow;
    hold off;
end