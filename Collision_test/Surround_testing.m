clc; clf;
%% testing with surroundings 
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

camlight;
hold on;
axis equal;
axis on;
%loading GUI
GUI = move_GUI;



%placing objects
PlaceObject('Conveyor_Belt.ply');
beam = placeply('beams_low.ply',0,0.3,0,0);
Table = placeply('Table.ply',0.5,0,0,pi/2);
Table2 = placeply('Table.ply',0.9,0,0,pi/2);
conveyor2 = placeply('Conveyor_Belt.ply',0,0,0,pi);
emptybox = placeply('box.ply',0,0.8,.4,0);
[bigboxVert,bigboxfaces,bigboxnormals,bigbox] = placeplyFV('BIG_BOX.ply',-1.5,0,0.38,0);
vertex = bigboxVert(:,1:3);
%generating robots
robot = UR3;
GUI.UR3 = robot.model;
robot.model.base = transl(0,-0.1,0.9)*trotx(pi);
robot.model.animate([0 0 0 0 0 0]);

kuka = LBRiiwa14R820;
GUI.KUKA = kuka.model;
kuka.model.base = transl(0,0.5,0.9)*trotx(pi);
kuka.model.animate(kuka.qz);

%setting up ray path for checking box position
startP = robot.model.base(1:3,4)';
starttr = transl(startP(1),startP(2),startP(3));
maxRange = 3;
rayAtOrigin = maxRange * -starttr(1:3,1)';
rayEnd = startP + rayAtOrigin;
yRotAxis = starttr(1:3,2)';

%creating a path for the box
for i = 1:100
    
    
    
    %moving bigbox along conveyor belt
    bigboxVert = bigboxVert*transl(0.03,0,0)';
    set(bigbox,'Vertices', bigboxVert(:,1:3));
    %getting box info for tracking
    bigboxfaces = get(bigbox,'Faces');
    vertex = bigboxVert(:,1:3);
    bigboxnormals = getfacenormals(vertex,bigboxfaces);
    
    for yRotRads = deg2rad(20):deg2rad(1):deg2rad(30)
        tr = makehgtform('axisrotate',yRotAxis,yRotRads);
        rayEnd = startP +  rayAtOrigin * tr(1:3,1:3)
        
        %check for intersection with box
        for faceIndex = 1:size(bigboxfaces,1)
            vertOnPlane = vertex(bigboxfaces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(bigboxnormals(faceIndex,:),vertOnPlane,startP,rayEnd);
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(bigboxfaces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
            end
        end
        
        
        
    end
pause(0.3);


end
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