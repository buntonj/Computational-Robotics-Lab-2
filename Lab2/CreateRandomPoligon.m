function obstacleVertices = CreateRandomPoligon(sides,roomBounds) 

x = roomBounds(1,1) + (-roomBounds(1,1)+roomBounds(1,2))*rand(1,1) ;
y = roomBounds(2,1)+(-roomBounds(2,1)+roomBounds(2,2))*rand(1,1) ;
x = x + roomBounds(1,2)/10*(rand(1,sides)-0.5) ;
y = y + roomBounds(1,2)/10*(rand(1,sides)-0.5) ;
P = [x; y]; % coordinates / points 
c = mean(P,2); % mean/ central point 
d = P-c ; % vectors connecting the central point and the given points 
th = -atan2(d(2,:),d(1,:)); % angle above x axis
[th, idx] = sort(th);   % sorting the angles 
P = P(:,idx); % sorting the given points

obstacleVertices = P(:,sort(unique(convhull(P'))))';
%P = [P P(:,1)]; % add the first at the end to close the polygon 
%plot( P(1,:), P(2,:), '.-r');