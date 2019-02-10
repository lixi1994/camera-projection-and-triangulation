%iterative algorithm to compute vicon radial distortion

function [newx, newy] = performRadialDistortion(xfilm, yfilm, radialparameters)

xorig = xfilm;
yorig = yfilm;
newx = xfilm;
newy = yfilm;
radk = radialparameters;
switch length(radk)  %make sure there are 3 radial distortion parameters
    case 0  
        radk = [0 0 0];
    case 1
        radk(2) = 0; radk(3) = 0;
    case 2
        radk(3) = 0;
end

for ii=1:10
    r2val = newx.^2 + newy.^2;
    %rtmp = (1 + radk(1)*r2val + radk(2)*(r2val.^2) + radk(3)*(r2val.^3));
    rtmp = (1 + r2val.*(radk(1) + r2val.*(radk(2) + r2val*radk(3))));
    xold = newx; yold = newy;
    newx = xorig./rtmp;
    newy = yorig./rtmp;
    resid = max(abs([newx-xold newy-yold]));
end

return