clearvars;
close all;
clc;
%%
x1 = linspace(0,6,100);
% diag line
y1 = x1;
path.rearaxle = [x1; y1].';
% uisave('path');
figure(1)
plot(path.rearaxle(:,1),path.rearaxle(:,2), 'bo');
%%
% circle midle of the lab
a1 = 3; b1=3; r1 = 2;
theta = linspace(0, 2*pi, 100);
x2 = a1 + r1*cos(theta);
y2 = b1 + r1*sin(theta);
path.rearaxle = [x2; y2].';
% uisave('path')
figure(2)
plot(path.rearaxle(:,1),path.rearaxle(:,2), 'bo');
axis equal ;
for k = 1:length(x2)
    text(x2(k), y2(k), num2str(k));
end
%%
% vertical line
y3 = linspace(0, 4, 5);
x3 = zeros(1,length(y3));
% 90 deg turn
a = 3; b = 4; r = 3;
theta = linspace(pi/2, pi, 5);
x4 = a + r*cos(theta);
y4 = b + r*sin(theta);
% horizontal line
x5 = linspace(3, 8, 6);
y5 = ones(1,length(x5)).*7;
figure(3)
plot(x5,y5, 'bo');
% concatenat x3, x4, x5
x6 = [x3 flip(x4(2:4)) x5];
y6 = [y3 flip(y4(2:4)) y5];
path.rearaxle = [x6; y6].';
uisave('path')
figure(3)
plot(path.rearaxle(:,1),path.rearaxle(:,2), 'bo');
for k = 1:length(path.rearaxle(:,1))
    text(path.rearaxle(k,1), path.rearaxle(k,2), num2str(k));
end

%%
% horizontal line
x7 = linspace(0, 8, 8);
y7 = zeros(1,length(x7));
figure(3)
plot(x7,y7, 'bo');
% path.rearaxle = [x7; y7].';
% uisave('path')

%%
%% Create sparse figure eight path
r = 3; % radius of each circle
n = 10; % number of points per arc (sparse)

% First half circle (counter-clockwise) - bottom loop
theta1 = linspace(pi/2, 3*pi/2, n); % left to right arc
x1 = -r + r * cos(theta1);
y1 = r * sin(theta1);

% Second half circle (clockwise) - top loop
theta2 = linspace(3*pi/2, pi/2, n); % right to left arc
x2 = r + r * cos(theta2);
y2 = r * sin(theta2);

% Connect loops at center point
x_mid = [0]; y_mid = [0];

% Concatenate full path
x = [x1, x_mid, flip(x2)];
y = [y1, y_mid, flip(y2)];

% Create rear axle path
path.rearaxle = [x; y].';

% Save path
% uisave('path')

% Plot
figure(4); clf;
plot(path.rearaxle(:,1), path.rearaxle(:,2), 'bo-'); hold on;
axis equal; grid on;
title('Sparse Figure Eight Path');

% Label points
for k = 1:length(path.rearaxle)
    text(path.rearaxle(k,1), path.rearaxle(k,2), num2str(k));
end

%% plt sigmoid
x8 = linspace(-8, 8, 17);
y8 = flip(6./(1+exp(-x8))-3);
% plot(x8,y8, 'b.-');
a1 = -8; b1 = 0; r1 = 3;
theta1 = linspace(pi/2, 3*pi/2, 11);
x9 = a1 + r1*cos(theta1);
y9 = b1 + r1*sin(theta1);
% plot(flip(x9(2:10)),flip(y9(2:10)));

x10 = linspace(-8, 8, 17);
y10 = 6./(1+exp(-x10))-3;
% plot(x10,y10);

a2 = 8; b2 = 0; r2 = 3;
theta2 = linspace(3*pi/2, 2*pi, 6);
x11 = a2 + r2*cos(theta2);
y11 = b2 + r2*sin(theta2);
% plot(flip(x9(2:5)),flip(y9(2:5)));

a3 = 8; b3 = 0; r3 = 3;
theta3 = linspace(0, pi/2, 5);
x12 = a3 + r3*cos(theta3);
y12 = b3 + r3*sin(theta3);
% plot(flip(x9(1:4)),flip(y9(1:4)));

x = [x10, flip(x12(1:4)), flip(x11(2:5)), flip(x8), x9(2:10)];
y = [y10, flip(y12(1:4)), flip(y11(2:5)), flip(y8), y9(2:10)];

path.rearaxle = [x; y].';
% uisave('path')
figure(10)
axis equal;
hold on;
plot(path.rearaxle(:,1),path.rearaxle(:,2), 'bo');
for k = 1:length(path.rearaxle(:,1))
    text(path.rearaxle(k,1), path.rearaxle(k,2), num2str(k));
end

%% ================== TruckLab diagnal path generation =====================
clc; clear; close all;

% Origin
x0 = 0; 
y0 = 0;

% Endpoints
end1 = [-1.97, -1.912];   % [x13, y13] for line 1
end2 = [1.911, 2.039];    % [x14, y14] for line 2

% Number of waypoints (including origin & endpoint)
N = 20;  % Increase for denser path

% Create waypoints for line 1
x13 = linspace(x0, end1(1), N);
y13 = linspace(y0, end1(2), N);

% Create waypoints for line 2
x14 = linspace(x0, end2(1), N);
y14 = linspace(y0, end2(2), N);

% Concatenate into a single path
x15 = [flip(x13), x14(2:end)]; %(1:end-1)
y15 = [flip(y13), y14(2:end)];

path.rearaxle = [x15; y15].';
uisave('path')
% Plot lines
figure(11); hold on; grid on; axis equal;
plot(path.rearaxle(:,1), path.rearaxle(:,2), 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Line 1 waypoints');
for k = 1:length(path.rearaxle(:,1))
    text(path.rearaxle(k,1), path.rearaxle(k,2), num2str(k));
end
% plot(x2, y2, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Line 2 waypoints');
% Mark origin
plot(x0, y0, 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'Origin');

% Labels
xlabel('X [m]');
ylabel('Y [m]');
title('Two Reference Paths from OptiTrack Origin');
legend show



