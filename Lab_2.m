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
UR3 = LinearUR3(false);
UR3_startQ = [0,0,-pi/2,0,-pi/2,0,0];
UR3.model.animate(UR3_startQ)
% Desired Roll = 180, Pitch = 0 (facing down)
hold on
KUKA = LBRiiwa14R820;

% startQ = zeros(1,7);
% UR3.model.animate(startQ)
% KUKA.model.animate(startQ)

%% Setting up grippers
% UR3 gripper (suction)
workspace = [-0.5 0.5 -0.5 0.5 0 0.5];
L1 = Link('d',0,'a',0,'alpha',0, 'offset',0);
UR3Grip = SerialLink([L1],'name','UR3Grip');
UR3Grip.base = UR3.model.fkine(UR3_startQ) * trotx(pi) * transl(0,0,-0.11);
[faceData,vertexData] = plyread('vacuumGripper.ply','tri');
UR3Grip.faces = {faceData, []};
UR3Grip.points = {vertexData, []};
plot3d(UR3Grip,0,'workspace',workspace);

% kuka gripper