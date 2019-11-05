function obstacles=AddObstacle(obstacleVertices,obstacles)
if nargin ~= 2 % if the number of inputs equals 3
    obstacles(1,1) = {obstacleVertices};
    % Compute obstacle face normals
    normals = zeros(length(obstacleVertices),3);
    for i = 1:length(obstacleVertices)
        if i== length(obstacleVertices)
            normals(i,1:2)=[-(obstacles{1,1}(1,2)-obstacles{1,1}(i,2)),(obstacles{1,1}(1,1)-obstacles{1,1}(i,1))];
            normals(i,1:2)=normals(i,1:2)/norm(normals(i,1:2));
            normals(i,3)=-normals(i,1:2)*obstacles{1,1}(i,:)';
        else
            normals(i,1:2)=[-(obstacles{1,1}(i+1,2)-obstacles{1,1}(i,2)),(obstacles{1,1}(i+1,1)-obstacles{1,1}(i,1))];
            normals(i,1:2)=normals(i,1:2)/norm(normals(i,1:2));
            normals(i,3)=-normals(i,1:2)*obstacles{1,1}(i,:)';
        end
    end
    obstacles(1,2) = {normals};
else
    indexes = size(obstacles);
    obstacles(indexes(1)+1,1)= {obstacleVertices};
    normals = zeros(length(obstacleVertices),3);
    for i = 1:length(obstacleVertices)
        if i== length(obstacleVertices)
            normals(i,1:2)=[(obstacles{indexes(1)+1,1}(1,2)-obstacles{indexes(1)+1,1}(i,2)),-(obstacles{indexes(1)+1,1}(1,1)-obstacles{indexes(1)+1,1}(i,1))];
            normals(i,1:2)=normals(i,1:2)/norm(normals(i,1:2));
            normals(i,3)=-normals(i,1:2)*obstacles{indexes(1)+1,1}(i,:)';
        else
            normals(i,1:2)=[(obstacles{indexes(1)+1,1}(i+1,2)-obstacles{indexes(1)+1,1}(i,2)),-(obstacles{indexes(1)+1,1}(i+1,1)-obstacles{indexes(1)+1,1}(i,1))];
            normals(i,1:2)=normals(i,1:2)/norm(normals(i,1:2));
            normals(i,3)=-normals(i,1:2)*obstacles{indexes(1)+1,1}(i,:)';
        end
    end
    obstacles(indexes(1)+1,2) = {normals};
end