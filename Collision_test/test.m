 
[bigboxVert,bigboxfaces,bigboxnormals,bigbox] = placeplyFV('BIG_BOX.ply',-1.5,0,0.38,0);
vertex = bigboxVert(:,1:3);
bigboxfacesstore = zeros(12,3,20);

for i = 1:20
bigboxVert = bigboxVert*transl(0.03,0,0)';
    set(bigbox,'Vertices', bigboxVert(:,1:3));
    %getting box info for tracking
    bigboxfaces = get(bigbox,'Faces');
    bigboxfacesstore(:,:,i) = bigboxfaces ;
    vertex = bigboxVert(:,1:3);
    bigboxnormals = getfacenormals(vertex,bigboxfaces);
end    