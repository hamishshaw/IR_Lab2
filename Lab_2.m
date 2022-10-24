clf; clear; close all; clc;
set(0,'DefaultFigureWindowStyle','docked');

%% Surface
surf([-2,-2;2,2],[-2,2.5;-2,2.5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

camlight;
hold on;
axis equal;
axis on;
% view(3);
GUI = move_GUI;

%% Loading the environment
conveyor1 = placeply('Conveyor_Belt.ply',0,0,0,0);
conveyor2 = placeply('Conveyor_Belt.ply',0,0,0,pi);
pallet = placeply('pallet.ply',0,0.3,0,0); % 0.142m high
beam = placeply('beams_safety_barrier_curtain_forklift.ply',0,0.3,0,0);
% beam = placeply('beams.ply',0,0.3,0,0);
Table = placeply('Table.ply',0.5,0,0,pi/2); % 0.4m high
Table2 = placeply('Table.ply',0.8,0,0,pi/2);

%% Place boxes
workspace = [-0.5 0.5 -0.5 0.5 0 0.5];
L1 = Link('d',0,'a',0,'alpha',0, 'offset',0);
% Box 1
box1 = SerialLink(L1,'name','box1'); % 0.2m high
box1.base = transl(0,1.05,0.142);
[faceData,vertexData] = plyread('cardboardBox.ply','tri');
box1.faces = {faceData, []};
box1.points = {vertexData, []};
box1.delay = 0;
plot3d(box1,0,'workspace',workspace);

% Box 2
box2 = SerialLink(L1,'name','box2');
box2.base = transl(0,1.3,0.142);
[faceData,vertexData] = plyread('cardboardBox.ply','tri');
box2.faces = {faceData, []};
box2.points = {vertexData, []};
box2.delay = 0;
plot3d(box2,0,'workspace',workspace);

% Box 3
box3 = SerialLink(L1,'name','box3');
box3.base = transl(0,1.55,0.142);
[faceData,vertexData] = plyread('cardboardBox.ply','tri');
box3.faces = {faceData, []};
box3.points = {vertexData, []};
box3.delay = 0;
plot3d(box3,0,'workspace',workspace);

% Box contents
small = SerialLink(L1,'name','small'); % 0.07m high
small.base = transl(0,0.65,0.4);
[faceData,vertexData] = plyread('smallerBox.ply','tri');
small.faces = {faceData, []};
small.points = {vertexData, []};
small.delay = 0;

%% Light Curtains for conveyor belt detection
% importing ply files
lightcurtain1 = placeply('Light_curtain.ply',.22,-1,0,pi/2);
lightcurtain2 = placeply('Light_curtain.ply',0.22,1,0,-pi/2);
% creating points for checking collisions
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
    linecurtain = plot3([LightStart(i+1,1),LightEnd(i+1,1)],[LightStart(i+1,2),LightEnd(i+1,2)],[LightStart(i+1,3),LightEnd(i+1,3)],'r');
end
collision = false;
collisionavoided = false;
maxzclear = 0;
boxcount = 0;


%% Setting up UR3 and Kuka LBR iiwa 14 R820 Robot
ur3 = LinearUR3(false);
ur3_startQ = [0 0 0 0 -pi/2 pi/2 0];
ur3.model.animate(ur3_startQ)
GUI.UR3 = ur3.model;
ur3.model.delay = 0;
% Desired Roll = 180, Pitch = 0, Yaw = 169 (facing down/desk)
% Desired Roll = 180, Pitch = 0, Yaw = -11 (facing down/box)
hold on
kuka = LBRiiwa14R820;
kuka_startQ = [pi/2 0 0 pi/2 0 pi/2 0];
kuka.model.base = transl(0,0.35,0.4);
kuka.model.delay = 0;
kuka.model.animate(kuka_startQ)
GUI.KUKA = kuka.model;

%% Setting up grippers
% UR3 gripper (suction), 0.12m high
L1 = Link('d',0,'a',0,'alpha',0, 'offset',0);
ur3Grip = SerialLink(L1,'name','ur3Grip');
ur3Grip.base = ur3.model.fkine(ur3_startQ) * trotx(pi) * transl(0,0,-0.11);
[faceData,vertexData] = plyread('vacuumGripper.ply','tri');
ur3Grip.faces = {faceData, []};
ur3Grip.points = {vertexData, []};
plot3d(ur3Grip,0,'workspace',workspace);
ur3Grip.delay = 0;
GUI.UR3Grip = ur3Grip;

% kuka gripper
kukaGrip = Gripper(kuka.model.fkine(kuka.model.getpos())); % Base approx 0.05m high
GUI.KUKAGrip = kukaGrip;

%% Moving the boxes
ur3.model.delay = 0;
ur3Grip.delay = 0;
kuka.model.delay = 0;
box1.delay = 0;

% Move to box 1
tr = box1.fkine(0) * transl(0,0,0.32);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC2(ur3, 10, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    drawnow();
end

% Lift box 1 directly up
tr = box1.fkine(0);
xyz2 = tr(1:3,4);
xyz2(3) = 0.75;
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 3, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    box1.base = ur3.model.fkine(qMatrix(i,:)) * trotz(pi) * transl(0,0,0.11);
    box1.animate(0)
    drawnow();
end

% Move to table
tr = transl(0,0.65,0.75);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 3, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    box1.base = ur3.model.fkine(qMatrix(i,:)) * trotz(pi) * transl(0,0,0.11);
    box1.animate(0)
    drawnow();
end

% Place on table
tr = transl(0,0.65,0.7);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 2, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    box1.base = ur3.model.fkine(qMatrix(i,:)) * trotz(pi) * transl(0,0,0.11);
    box1.animate(0)
    drawnow();
end

% Move away from box
tr = transl(0,0.65,0.75);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 3, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    drawnow();
end

pause(2);
plot3d(small,0,'workspace',workspace);
small.delay = 0;

% Move to box again
tr = transl(0,0.65,0.7);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 2, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    drawnow();
end

% Lift box directly up
tr = transl(0,0.65,0.8);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(ur3, 2, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    box1.base = ur3.model.fkine(qMatrix(i,:)) * trotz(pi) * transl(0,0,0.11);
    box1.animate(0)
    drawnow();
end

% Move box towards bin
tr = transl(-0.5,0.5,0.8);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 90]);
[qMatrix, steps] = RMRC(ur3, 5, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    ur3.model.animate(qMatrix(i,:))
    ur3Grip.base = ur3.model.fkine(qMatrix(i,:)) * trotx(pi) * transl(0,0,-0.11);
    ur3Grip.animate(0)
    box1.base = ur3.model.fkine(qMatrix(i,:)) * trotz(pi) * transl(0,0,0.11);
    box1.animate(0)
    drawnow();
end

% Using the KUKA now
% Go above small box
tr = small.fkine(0) * transl(0,0,0.3);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
openGripper(kukaGrip);
[qMatrix, steps] = RMRC(kuka, 10, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
    kuka.model.animate(qMatrix(i,:))
    drawnow();
end

% Go to small box
tr = small.fkine(0) * transl(0,0,0.15);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
openGripper(kukaGrip);
[qMatrix, steps] = RMRC(kuka, 5, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
    kuka.model.animate(qMatrix(i,:))
    drawnow();
end

closeGripper(kukaGrip);
% Pick up small box
tr = small.fkine(0);
xyz2 = tr(1:3,4);
xyz2(3) = 0.62;
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(kuka, 3, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    kuka.model.animate(qMatrix(i,:))
    gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
    small.base = kuka.model.fkine(qMatrix(i,:)) * transl(0,0,0.08);
    small.animate(0)
    drawnow();
end

% Halfway to conveyor belt
tr = transl(0.3,0.35,0.55);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 90]);
[qMatrix, steps] = RMRC(kuka, 5, xyz2, theta2);
for i=1:steps
    checkEStop(GUI);
    kuka.model.animate(qMatrix(i,:))
    gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
    small.base = kuka.model.fkine(qMatrix(i,:)) * transl(0,0,0.08);
    small.animate(0)
    drawnow();
end

% Go to conveyor belt
tr = transl(0,0,0.55);
xyz2 = tr(1:3,4);
theta2 = deg2rad([180 0 0]);
[qMatrix, steps] = RMRC(kuka, 5, xyz2, theta2);
% for i=1:steps
%     checkEStop(GUI);
%     kuka.model.animate(qMatrix(i,:))
%     gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
%     small.base = kuka.model.fkine(qMatrix(i,:)) * transl(0,0,0.08);
%     small.animate(0)
%     drawnow();
% end
for i=1:steps
    checkEStop(GUI);
    %generate box at 50 steps
    if i == 100
        [bigboxVert,bigboxfaces,bigboxnormals,bigbox] = placeplyFV('BIG_BOX.ply',1.5,0,0.38,0);
        vertex = bigboxVert(:,1:3);
    end
    %start checking for collisions
    if i > 100 && collision == false && collisionavoided == false
        bigboxVert = bigboxVert*transl(-0.03,0,0)';
        set(bigbox,'Vertices', bigboxVert(:,1:3));
        bigboxfaces = get(bigbox,'Faces');
        vertex = bigboxVert(:,1:3);
        boxcount = boxcount+1;
        bigboxnormals = getfacenormals(vertex,bigboxfaces);
        % checking each beam for collisions with each face
        for n = 1:21
            for faceIndex = 1:size(bigboxfaces,1)
                vertOnPlane = vertex(bigboxfaces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(bigboxnormals(faceIndex,:),vertOnPlane,LightStart(n,:),LightEnd(n,:));
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(bigboxfaces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    collision = true;
                    display('Incoming Box');
                end
            end
        end
    end
     % moving the arm out of the way and then returning to position when safe
    if collision == true && collisionavoided == false
        %create path to move arm out of the way
        tr = kuka.model.fkine(qMatrix(1,:));
        xyzcollision = tr(1:3,4);
        theta2 = deg2rad([180 0 0]);
        [qMatrixmoveaway, stepscollision] = RMRC(kuka, 2, xyzcollision, theta2);
        
        for p = 1:stepscollision
            checkEStop(GUI);
            %move the arm
            kuka.model.animate(qMatrixmoveaway(p,:));
            gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
            small.base = kuka.model.fkine(qMatrixmoveaway(p,:)) * transl(0,0,0.08);
            small.animate(0)
            drawnow();
            %also move the box
            if boxcount < 100 && mod(p,2)==0
                bigboxVert = bigboxVert*transl(-0.03,0,0)';
                set(bigbox,'Vertices', bigboxVert(:,1:3));
                boxcount = boxcount+1;
                
            end
            
        end
      
        %move the arm back to previous paused goal
        tr = kuka.model.fkine(qMatrix(i,:));
        xyzcollision = tr(1:3,4);
        theta2 = deg2rad([180 0 0]);
        [qMatrixmoveaway, stepscollision] = RMRC(kuka, 2, xyzcollision, theta2);
        
        for p = 1:stepscollision
            % move kuka, gripper and box
            kuka.model.animate(qMatrixmoveaway(p,:))
            gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
            small.base = kuka.model.fkine(qMatrixmoveaway(p,:)) * transl(0,0,0.08);
            small.animate(0)
            drawnow();
            % move box
            if boxcount < 100 && mod(p,2)==0
                bigboxVert = bigboxVert*transl(-0.03,0,0)';
                set(bigbox,'Vertices', bigboxVert(:,1:3));
                boxcount= boxcount+1;
            end
        end
        collisionavoided = true;
        delete(bigbox);
    end
    
    %animating kuka when no collision detected
 
        kuka.model.animate(qMatrix(i,:))
        gripperAnimate(kukaGrip, kuka.model.fkine(kuka.model.getpos()));
        small.base = kuka.model.fkine(qMatrix(i,:)) * transl(0,0,0.08);
        small.animate(0)
        drawnow();
 
    
end
openGripper(kukaGrip);
%% RMRC function for no change in roll and pitch
function [qMatrix, steps] = RMRC(name, time, xyz2, theta2)
    deltaT = 0.02;      % Control frequency
    steps = time/deltaT;   % No. of steps for simulation
    epsilon = 0.3;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
    
    % Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,7);       % Array for joint anglesR
    qdot = zeros(steps,7);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    
    q = name.model.getpos;
    tr = name.model.fkine(q);
    xyz1 = tr(1:3,4);
    theta1 = tr2rpy(tr);
    
    % Set up trajectory, initial pose
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    for i=1:steps
        for j=1:3
            x(j,i) = (1-s(i))*xyz1(j) + s(i)*xyz2(j); % XYZ points
        end
        theta(1,i) = pi; % Roll
        theta(2,i) = 0; % Pitch
        theta(3,i) = (1-s(i))*theta1(j) + s(i)*theta2(j); % Yaw
    end
     
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = name.model.getpos;                                                            % Initial guess for joint angles
    qMatrix(1,:) = name.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
    
    % Track the trajectory with RMRC
    for i = 1:steps-1
        T = name.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = name.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:7                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < name.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > name.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    end
end

%% RMRC function if there is a change in roll and pitch
function [qMatrix, steps] = RMRC2(name, time, xyz2, theta2)
    deltaT = 0.02;      % Control frequency
    steps = time/deltaT;   % No. of steps for simulation
    epsilon = 0.3;      % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
    
    % Allocate array data
    m = zeros(steps,1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps,7);       % Array for joint anglesR
    qdot = zeros(steps,7);          % Array for joint velocities
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    x = zeros(3,steps);             % Array for x-y-z trajectory
    
    q = name.model.getpos;
    tr = name.model.fkine(q);
    xyz1 = tr(1:3,4);
    theta1 = tr2rpy(tr);
    
    % Set up trajectory, initial pose
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    for i=1:steps
        for j=1:3
            x(j,i) = (1-s(i))*xyz1(j) + s(i)*xyz2(j); % XYZ points
            theta(j,i) = (1-s(i))*theta1(j) + s(i)*theta2(j); % RPY angles
        end
    end
     
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = name.model.getpos;                                                            % Initial guess for joint angles
    qMatrix(1,:) = name.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
    
    % Track the trajectory with RMRC
    for i = 1:steps-1
        T = name.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
        S = Rdot*Ra';                                                           % Skew symmetric!
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
        xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
        J = name.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
        for j = 1:7                                                             % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < name.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > name.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    end
end

%% E-stop function
function checkEStop(GUI)
    while GUI.StopCheck == true
        if GUI.StopCheck == false
            break
        end
        pause(.1);
    end
end

%% is Intersecting with triangle
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end