%get face noramls
function faceNormals = getfacenormals(vertex,face)
faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
end