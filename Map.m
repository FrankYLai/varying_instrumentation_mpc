function map = Map(configs)
    canvas = configs("canvas");
    w_canvas = canvas(2) - canvas(1);
    h_canvas = canvas(4) - canvas(3);
    if configs("map") == 0 %empty map
        map = [];
    elseif configs("map") == 1
        num_obs = 3;
        map = zeros(2,4,num_obs);
        %center rectangle points
        [pts, radius] = Obstacle.rectangular([0,0,0], [5,1]);
        map(:,:,1) = pts;
        [pts, radius] = Obstacle.rectangular([-4,-4,0], [5,1]);
        map(:,:,2) = pts;
        [pts, radius] = Obstacle.rectangular([-4,4,0], [5,1]);
        map(:,:,3) = pts;
    end
    
end

