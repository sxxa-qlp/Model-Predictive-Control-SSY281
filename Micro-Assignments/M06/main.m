clear all; close all; clc;

figure('Color','white'), hold on, grid on;
ylim([-1 10])
plot([0,0],[-1,10],'LineWidth',3)
plot([-10,2],[11,-1],'LineWidth',3)

x = linspace(-10,2);
y = linspace(-1,10);
[X,Y] = meshgrid(x,y);
Z = X^2+Y^2;
contour(X,Y,Z,20)

legend({'h(x)','g(x)','f(x)'})
xlabel 'x1', ylabel 'x2'