clear;
clc;
poly1 = polyshape([0 0 1 1],[1 0 0 1]);
% poly1 = polyshape([-1 -1 0 0],[1 0 0 1]);
poly2 = polyshape([0.75 1.25 1.25 0.75],[0.25 0.25 0.75 0.75]);
plot(poly1)
hold on
plot(poly2)
xlim([-0.2 1.4]);
ylim([-0.2 1.2]);

polyout = intersect(poly1,poly2);
% area(polyout)

figure(2)
plot(polyout)
xlim([-0.2 1.4]);
ylim([-0.2 1.2]);