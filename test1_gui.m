clc 
clf
%% A file to show how the GUI can be used
%starting GUI app
GUI = move_GUI;

%setting up robot, these robots must also set some properties of the GUI to
%work
kuka = LBRiiwa14R820;   
kuka.model.base =  transl(1,1,0);
kuka.model.animate([0 0 0 0 0 0 0])
hold on
UR3 = LinearUR3(false);
ur3_startQ = [0 0 0 0 -pi/2 pi/2 0];
UR3.model.animate(ur3_startQ)
GUI.KUKA = kuka.model;
GUI.UR3 = UR3.model;

%gripper test
L1 = Link('d',0,'a',0,'alpha',0, 'offset',0);
ur3Grip = SerialLink([L1],'name','ur3Grip');
ur3Grip.base = ur3.model.fkine(ur3_startQ) * trotx(pi) * transl(0,0,-0.11);
[faceData,vertexData] = plyread('vacuumGripper.ply','tri');
ur3Grip.faces = {faceData, []};
ur3Grip.points = {vertexData, []};
plot3d(ur3Grip,0,'workspace',workspace);
ur3Grip.delay = 0;
GUI.UR3Grip = ur3Grip;

kukaGrip = Gripper(kuka.model.fkine(kuka.model.getpos()));
GUI.KUKAGrip = kukaGrip;

%making some path
steps = 50;
qstart = [-pi/4 pi/2 -pi/4 pi/2 pi/4 pi/4 pi/2];
qend = zeros(1,UR3.model.n);
qmatrix = jtraj(qstart,qend,steps);
UR3.model.base = transl(0,0,0)*trotx(pi/2);
UR3.model.animate(qstart);
pause(5);

%testing to see how estop can be integrated
%i have tried to get this while loop thing that holds the code in a loop
%until the button is pressed again to go into a function however have had
%troubles then getting it to work, will continue work on this. I will also
%try think of some "cool innovations" that it can do.
for n = 1:steps
%estop check    
while GUI.StopCheck == true % e-stop function
    if GUI.StopCheck == false
        break
    end
    pause(.1);
end


UR3.model.animate(qmatrix(n,:));
n
pause(.1);
end

%testing to see how the x and y adjustment works

