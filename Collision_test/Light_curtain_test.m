% clf; clear; close all; clc;
% set(0,'DefaultFigureWindowStyle','docked');
% 
% % Surface
% surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% xlabel('x');
% ylabel('y');
% camlight;
% hold on;
% axis equal;
% axis on;
% % Loading the environment
% conveyor1 = placeply('Conveyor_Belt.ply',0,0,0,0);
% conveyor2 = placeply('Conveyor_Belt.ply',0,0,0,pi);
% pallet = placeply('pallet.ply',0,0.3,0,0); % 0.142m high
% beam = placeply('beams.ply',0,0.3,0,0);
% Table = placeply('Table.ply',0.5,0,0,pi/2); % 0.4m high
% Table2 = placeply('Table.ply',0.8,0,0,pi/2);
% 
% lightcurtain1 = placeply('Light_curtain.ply',.22,-1,0,pi/2);
% lightcurtain2 = placeply('Light_curtain.ply',0.22,1,0,-pi/2);

%% Light beam generation

startP = [1,0.25,0.4];
starttr = transl(startP(1),startP(2),startP(3));
maxRange = 0.5;
rayAtOrigin = maxRange * -starttr(1:3,2)';
rayEnd = startP + rayAtOrigin;
LightStart = zeros(21,3);
LightEnd = zeros(21,3);
for i = 0:20
    LightStart(i+1,:) = startP+[0 0 i*0.03];
    LightEnd(i+1,:) = rayEnd+[0 0 i*0.03];
end
