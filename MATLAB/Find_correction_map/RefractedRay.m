
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) MERL 2012
% CVPR 2012 Paper Title: A Theory of Multi-Layer Flat Refractive Geometry
% Author: Amit Agrawal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Compute refracted ray direction at a refraction boundary


function [vr,a,b,tir] = RefractedRay(vi,n,mu1,mu2)

tir = 0;


n = n/norm(n);

kk = mu1^2*(vi'*n)^2 - (mu1^2-mu2^2)*(vi'*vi);

if(kk < 0)
    % total internal reflection
    tir = 1;
    a = 0;
    b = 0;
    vr = zeros(3,1);
    return
end

a = mu1/mu2;
b = -mu1*(vi'*n) - sqrt(kk);
b = b/mu2;

vr = a*vi + b*n;





