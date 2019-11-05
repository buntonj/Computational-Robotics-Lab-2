%%credit http://enginius.tistory.com/

rszwh  = [75 100];

carwh  = [0.45 0.18];
[rcarrsz, rtrrsz] = get_rszcarimg('car.png', rszwh);
% POSITIONS
x=rand(1);
rcarpos = [x sin(x)+0.5 360*cos(sin(x))];
% LOAD IMAGES





% THIS HAS TO RUN EVERY TIME. 

% PLOT
axis tight manual
figure(); hold on; %set(gcf,'Color', [0.6, 0.9, 0.8]/4 );
plot(0:0.01:5,sin(0:0.01:5))
plot(0:0.01:5,sin(0:0.01:5)+1)
plot_carimage(rcarpos, carwh, rszwh, rcarrsz, rtrrsz); 

axis equal ; grid on; 

xlabel('X'); ylabel('Y');

title('Plot Cars', 'FontSize', 15, 'Color', 'w');
axis([0 5 -1 2])

