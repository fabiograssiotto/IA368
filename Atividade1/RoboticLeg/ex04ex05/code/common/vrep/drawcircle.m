% Prepare Graph
title('Comparação de Trajetórias');
xlabel('X');
ylabel('Y');

% Center and Radius for the circle.
x = 0;
y = -2;
r = 0.5;

hold on
% Robot Circle
xRobot = export(:,2);
yRobot = export(:,4);
plot(xRobot, yRobot);

% Base Circle with same center and radius
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit);
hold off

