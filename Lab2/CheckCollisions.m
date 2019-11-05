function collision = CheckCollisions(robotCornerTrajectories, obstacles)

collision = 0;
cornerPositions = [ [robotCornerTrajectories{1};robotCornerTrajectories{2};robotCornerTrajectories{3};robotCornerTrajectories{4}] ones(4*length(robotCornerTrajectories{1}),1)]';
[r,c] = size(cornerPositions);
i     = 1:numel(cornerPositions);
j     = repmat(1:c,r,1);
cornerPositionsMatrix = sparse(i',j(:),cornerPositions(:));
statesToCheck=length(cornerPositions);
for j=1:length(obstacles)
    facesToCheck = length(obstacles{j,2});
    obstacleFacesNormals = repmat(obstacles{j,2},statesToCheck,1);
    [A,B] = find(kron(speye(statesToCheck),ones(facesToCheck,3)));
    AMatrix = sparse(A,B,reshape(permute(reshape(obstacleFacesNormals, facesToCheck, statesToCheck, 3), [2 1 3]), statesToCheck, facesToCheck*3)');
    
    % % Multiplicacion en sparse
    InOut = nonzeros(AMatrix*cornerPositionsMatrix);
    InOutOrdered=InOut(1:facesToCheck:end);
    for i=2:facesToCheck
        InOutOrdered=[InOutOrdered InOut(i:facesToCheck:end)];
    end
    stateCollision = InOutOrdered<0;
    if j==1
        collision = collision + sum(sum(~stateCollision));
    else
        collision = collision + max(sum(stateCollision,2)==0); % Si sumo seis estoy adentro de todas las caras
    end
end