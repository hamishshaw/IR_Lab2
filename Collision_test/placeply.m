%place .ply at xyz
function [transformedV,faces,mesh] = placeply(plyfile,x,y,z,rotation)
mesh = PlaceObject(plyfile);
vertices = get(mesh,'Vertices');
faces = get(mesh,'Faces');
transformedV= [vertices,ones(size(vertices,1),1)] * transl(x,y,z)'*trotz(rotation)';
set(mesh,'Vertices',transformedV(:,1:3));

end