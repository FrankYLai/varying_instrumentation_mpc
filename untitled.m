canvas = [-2 2 -2 2]
[X, Y] = meshgrid(-2:0.1:2, -2:0.1:2);
X_flat = reshape(X,[],1);
Y_flat = reshape(Y,[],1);
pts = [X_flat,Y_flat];

v1 = [0,1];
v2 = [0,-1];
v3 = [1, 1];

verts = [v1;v2;v3];


costmap = [];
for pt = pts'
    c = cost_dist(pt, verts);
    if isempty(costmap)
        disp("empty")
        costmap = c;
    else
        costmap = [costmap;c];
    end
end

costmap = reshape(costmap,size(X));
figure(1)
surf(X,Y,costmap)

figure(2)
hold on;
fill(verts(:,1), verts(:,2), 'k');
scatter(pt(1), pt(2));
axis(canvas);


point_to_line(pt, v1, v2)

function dist = point_to_line(pt, v1, v2)
    pt = reshape(pt,1,2);
    a = v1 - v2;
    b = pt - v2;
    line_vec = a ;%vector(start, end) # (3.5, 0, -1.5)
    pnt_vec = b ;%vector(start, pnt)  # (1, 0, -1.5)
    line_len = sqrt(sum(line_vec.^2)); % # 3.808
    line_unitvec = line_vec/line_len; % # (0.919, 0.0, -0.394)
    pnt_vec_scaled = pnt_vec/line_len; %  # (0.263, 0.0, -0.393)
    t = dot(line_unitvec, pnt_vec_scaled); % # 0.397
    if t < 0.0
        t = 0.0;
    elseif t > 1.0
        t = 1.0;
    end
    nearest = line_vec* t; %    # (1.388, 0.0, -0.595)
    dist = sqrt(sum((nearest-pnt_vec).^2));% # 0.985
end

function c = cost_dist(pt, obs)
    mindist = 100;
    for k = 1:size(obs,1)  
        pt1 = obs(k,:);
        pt2 = obs(mod(k,size(obs,1))+1,:);
        d = point_to_line(pt, pt1, pt2);
        if d<mindist
            mindist = d;
        end
    end
    c = 2/(mindist^2 + 1);

end