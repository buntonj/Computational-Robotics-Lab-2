function plot_carimage(carpos, carwh, rszwh, carrsz, trrsz) 



[xmesh_grid, ymesh_grid] = get_carmeshgred(carpos, carwh, rszwh);

h = surf('xdata', xmesh_grid, 'ydata', ymesh_grid, 'zdata', zeros(rszwh(1), rszwh(2)), 'cdata', carrsz, 'AlphaData', trrsz , 'FaceAlpha', 'texture', 'FaceColor', 'texturemap' , 'EdgeColor','None', 'LineStyle', 'None');