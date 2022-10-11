clf; clear; close all; clc;

%% Setting up UR3 and Kuka LBR iiwa 14 R820 Robot
UR3 = LinearUR3(false);
hold on
kuka = LBRiiwa14R820;

kuka.model.base = kuka.model.base * transl(-1,1.3,0);
startQ = zeros(1,7);
UR3.model.animate(startQ)
kuka.model.animate(startQ)

%% Setting up grippers
% UR3 gripper (suction)


% kuka gripper