function [mu_long, mu_lat] = tire_model(Fz, slip_ang, p)
%tire_model.m Returns longitudinal and lateral friction coefficient
% Inputs:
%   Fz      : normal downwards force on the tire (lb)
%   slip_ang: slip angle of the tire (deg), default 0 deg
%   p       : tire pressure (psi), default 10 psi
%
% Outputs: 
%   mu_long : longitudinal friction coefficient
%   mu_lat  : lateral friction coefficient.

if ~exist('slip_ang', 'var')
    slip_ang = 0;
end

if ~exist('p', 'var')
    p = 10;
end


mu_long = 0.9;
mu_lat = 0.9;

end