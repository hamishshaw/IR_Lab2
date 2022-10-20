%placeplyFV testing
clf
clc
%creating robot
robot = SchunkUTSv2_0();
q = [0,pi/2,0,0,0,0];
robot.plot3d(q);
view(3);
camlight;
hold on;
pause(3);
%setting up transform
tr = robot.fkine(q);
startP = tr(1:3,4)';
maxRange = 3;
%generating box
[transformedV,faces,faceNormals,mesh] = placeplyFV('BIG_BOX.ply',0,1.5,0.30,0);
vertex = transformedV(:,1:3);
%setting ray path
rayAtOrigin = maxRange * tr(1:3,3)';
rayEnd = startP +  rayAtOrigin 
%  [intersectP,check] = LinePlaneIntersection(triangleNormal,trianglePoint,startP,rayEnd);

 for faceIndex = 1:size(faces,1)
    vertOnPlane = vertex(faces(faceIndex,1)',:);
    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,startP,rayEnd);
    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
        plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
        display('Intersection');
    end
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