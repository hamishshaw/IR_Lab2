%% placeply + facenormals 
function [transformedV,face,faceNormals,mesh] = placeplyFV(plyfile,x,y,z,rotation)
mesh = PlaceObject(plyfile);
vertices = get(mesh,'Vertices');

transformedV= [vertices,ones(size(vertices,1),1)] * transl(x,y,z)'*trotz(rotation)';
set(mesh,'Vertices',transformedV(:,1:3));
vertex = transformedV(:,1:3);
face = get(mesh,'Faces');

  faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end

end