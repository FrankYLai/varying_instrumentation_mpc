function animate(car, dynamic, map, path)
    % draw map
    
    %fill(xpoints+pos, ypoints, 'r');
    
    for obs = dynamic %draw dynamic obstacles
        [verticies, radius] = obs.gen_verticies(obs.currnet_pose, obs.dims);
        disp(verticies)
        
    end
    
    axis([-12 2 -5 5])
    drawnow
end