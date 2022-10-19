clf; clear; close all; clc;
set(0,'DefaultFigureWindowStyle','docked');

%% Surface
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

camlight;
hold on;
axis equal;
axis on;
% view(3);

%% Loading Conveyor Belt
PlaceObject('Conveyor_Belt.ply');

%% Loading Table
PlaceObject('Table.ply');

%% Loading Beam
PlaceObject('Beams.ply');

%% Setting up UR3 and Kuka LBR iiwa 14 R820 Robot
% Robot_UR3 = UR3;
% hold on
Robo_Kuka = LBRiiwa14R820;

% startQ = zeros(1,7);
% UR3.model.animate(startQ)
% kuka.model.animate(startQ)

%% Setting up grippers
% UR3 gripper (suction)


% kuka gripper