
%correct nonlinear lens distortion using vicon radial parameters

function [filmx, filmy] = performRadialCorrection(xdistorted, ydistorted, radialparameters)

radk = radialparameters;

switch length(radk)  %make sure there are 3 radial distortion parameters
    case 0  
        radk = [0 0 0];
    case 1
        radk(2) = 0; radk(3) = 0;
    case 2
        radk(3) = 0;
end

r2val = xdistorted.^2 + ydistorted.^2;
rtmp = (1 + r2val.*(radk(1) + r2val.*(radk(2) + r2val*radk(3))));
filmx = xdistorted.*rtmp;
filmy = ydistorted.*rtmp;

return
