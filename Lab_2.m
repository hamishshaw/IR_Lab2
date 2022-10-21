clf; clear; close all; clc;
set(0,'DefaultFigureWindowStyle','docked');

%% Surface
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

camlight;
hold on;
axis equal;
axis on;
% view(3);

%% Loading the environment
PlaceObject('Conveyor_Belt.ply');
beam = placeply('beams_low.ply',0,0.3,0,0);
Table = placeply('Table.ply',0.5,0,0,pi/2);
Table2 = placeply('Table.ply',0.9,0,0,pi/2);
conveyor2 = placeply('Conveyor_Belt.ply',0,0,0,pi);


%% Setting up UR3 and Kuka LBR iiwa 14 R820 Robot
ur3 = LinearUR3(false);
ur3_startQ = zeros(1,7);
ur3.model.animate(ur3_startQ)
% Desired Roll = 180, Pitch = 0 (facing down)
hold on
kuka = LBRiiwa14R820;
kuka_startQ = [0 -pi/2 0 pi/2 0 0 0];
kuka.model.base = transl(0,0,0.9)*trotx(pi);
kuka.model.animate(kuka_startQ)


% startQ = zeros(1,7);
% UR3.model.animate(startQ)
% KUKA.model.animate(startQ)

%% Setting up grippers
% UR3 gripper (suction)
workspace = [-0.5 0.5 -0.5 0.5 0 0.5];
L1 = Link('d',0,'a',0,'alpha',0, 'offset',0);
ur3Grip = SerialLink([L1],'name','ur3Grip');
ur3Grip.base = ur3.model.fkine(ur3_startQ) * trotx(pi) * transl(0,0,-0.11);
[faceData,vertexData] = plyread('vacuumGripper.ply','tri');
ur3Grip.faces = {faceData, []};
ur3Grip.points = {vertexData, []};
plot3d(ur3Grip,0,'workspace',workspace);

% kuka gripper