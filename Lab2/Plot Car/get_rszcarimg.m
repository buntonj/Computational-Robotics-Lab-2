function [carrsz, trrsz] = get_rszcarimg(imgpath, rszwh)

% GET RESIZED CAR IMAGE



[carimg, ~, cartr] = imread(imgpath);

carrsz = imresize(carimg, rszwh);

trrsz  = imresize(cartr, rszwh);