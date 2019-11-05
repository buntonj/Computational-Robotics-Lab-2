function dy = TricycleModel(t,y,goal)
global u
dy = zeros(3,1);
r = 0.025;
w = 0.090;
Gain1 = 30/r;
Gain2 = 30*w/r;
reverse = false;
Distance = goal(1:2)-[y(1);y(2)];
angleGoal=atan2(goal(2)-y(2),goal(1)-y(1));
if isnan(angleGoal)
    auxInputs(2) = 0;
elseif abs(angleGoal-y(3))<=pi/2
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
elseif angleGoal-y(3)>pi/2
    reverse = true;
    angleGoal = angleGoal-pi;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
else
    reverse = true;
    angleGoal = angleGoal+pi;
    auxInputs(2) = Gain2*sign(angleGoal-y(3))*abs(angleGoal-y(3));
end
auxInputs(2) = sign(auxInputs(2))*min(abs(auxInputs(2)),120);
auxInputs(1) = min(norm(Gain1*Distance),120-abs(auxInputs(2)));
if     reverse
    auxInputs(1)=-auxInputs(1);
end
inputs(1)=(auxInputs(1)+auxInputs(2))/2;
inputs(2)=(auxInputs(1)-auxInputs(2))/2;
if(isempty(u)||t==0)
    u=[0,0,0];
else
    u=[u;[inputs(1),inputs(2),t]];
end
RPMtoRadiansPerSecond = 2*pi/60;
dy(1) = r/2*RPMtoRadiansPerSecond*(inputs(1)+inputs(2))*cos(y(3));
dy(2) = r/2*RPMtoRadiansPerSecond*(inputs(1)+inputs(2))*sin(y(3));
dy(3)=r/w*RPMtoRadiansPerSecond*(inputs(1)-inputs(2));
