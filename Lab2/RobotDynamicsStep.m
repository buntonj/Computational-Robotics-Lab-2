function [newState,trajectory,u] = RobotDynamicsStep(state,goal,time)
global u
time=1;
u = [];
[t,trajectory] = ode45(@(t,y) TricycleModel(t,y,goal), [0,time], state);
% plot(trajectory(:,1),trajectory(:,2))


trajectory = interp1(t,trajectory,linspace(0,time,100*time)');

u=flip(u);
[C,ia,ic] = unique(u(:,3));
u = interp1(u(ia,3),u(ia,1:2),linspace(0,time,length(trajectory))');
newState = trajectory(end,:);
