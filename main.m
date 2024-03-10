configs = containers.Map;
configs("canvas") = [-12 12 -10 10];
configs("map") = 1;
configs("start") = [-10,0,-pi/2];
configs("end") = [10,0];

obstacle = Obstacle("rectangle_example",[-3,-2,pi/3],'rectangle',[0.4,0.2],[]);
obs2 = Obstacle("circle_example",[6,-6,0],'circle',[0.3],[]);
obs3 = Obstacle("triangle_example",[5,6,pi/2],'triangle',[0.3,0.4],[]);

map = Map(configs);

animate(configs, [], [obstacle, obs2, obs3], map, [])