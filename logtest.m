radius  = 1 : 1000;
% radius_norm = exp(radius*1e-2);
% radius_norm = 1 ./ (1 + exp(-radius));
radius_norm = 1e-3 *  radius.^2;


plot(radius, radius, radius, radius_norm )
% ylim([0, 1000])