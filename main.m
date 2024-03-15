configs = containers.Map;
configs("scene") = 1;

if configs("scene") == 1
    [obstacles, map, configs] = setup_scene1(configs);
end

%start simulation
% tic;
% while toc<40
% %     disp(toc);
%     animate(configs, [], obstacles, map, [], toc);
% end

for t = 1:0.05:10
    animate(configs, [], obstacles, map, [], t);
end