

% Copyright 2009 Mitsubishi Electric Research Laboratories All Rights Reserved.
% 
% Permission to use, copy and modify this software and its documentation without fee for educational, research and non-profit purposes, is hereby granted, provided that the above copyright notice and the following three paragraphs appear in all copies.
% 
% To request permission to incorporate this software into commercial products contact: Vice President of Marketing and Business Development; Mitsubishi Electric Research Laboratories (MERL), 201 Broadway, Cambridge, MA 02139 or <license@merl.com>.
% 
% IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
% 
% MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS. 
% 


% Solve Forward projection equation for Case 3 (two layer) (Air-Medium1-Medium2)
% d is the distance of medium1 from camera
% d2 is the total distance of medium2 from camera

% p is given 3D point
% n is the normal
% mu is refractive index vector

% M is the 3D point on the layer closest to camera where the first
% refraction happens



function [M] = SolveForwardProjectionCase3(d,d2,n,mu,p)


mu1 = mu(1,1);
mu2 = mu(2,1);

M = [0;0;1];


%find POR the plane of refraction
POR = cross(n,p);
POR = POR/norm(POR);

% [z1,z2] defines a coordinate system on POR
% axis is away from the camera. z1 is along the axis
z1 = -n;
z1 = z1/norm(z1);

% find z2
z2 = cross(POR,z1);
z2 = z2/norm(z2);

% find the projection of given 3D point on POR
v = p'*z1;
u = p'*z2;



%solve 12thth degree equation
s1 = (mu1^2 - 1)^2*(mu2^2 - 1)^2;

s2 = (-4)*u*(mu1^2 - 1)^2*(mu2^2 - 1)^2;

s3 = 4*u^2*(mu1^2 - 1)^2*(mu2^2 - 1)^2 + 2*(mu1^2 - 1)*(mu2^2 - 1)*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1));

s4 = - 2*(mu1^2 - 1)*(mu2^2 - 1)*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1)) - 4*u*(mu1^2 - 1)*(mu2^2 - 1)*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1));

s5 = ((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1))^2 + 2*(mu1^2 - 1)*(mu2^2 - 1)*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1)) - 4*(mu1^2 - 1)*(mu2^2 - 1)*(d - d2)^2*(d2 - v)^2 + 4*u*(mu1^2 - 1)*(mu2^2 - 1)*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1));

s6 = -2*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1))*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1)) - 4*u*(mu1^2 - 1)*(mu2^2 - 1)*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1)) - 4*d^4*mu1^2*mu2^2*u*(mu1^2 - 1)*(mu2^2 - 1);

s7 = 2*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1))*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1)) + (2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1))^2 - 4*(d - d2)^2*(d^2*mu1^2*(mu2^2 - 1) + d^2*mu2^2*(mu1^2 - 1))*(d2 - v)^2 + 10*d^4*mu1^2*mu2^2*u^2*(mu1^2 - 1)*(mu2^2 - 1);

s8 = -2*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1))*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1)) - 4*d^4*mu1^2*mu2^2*u*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1)) - 4*d^4*mu1^2*mu2^2*u^3*(mu1^2 - 1)*(mu2^2 - 1);

s9 = (d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1))^2 + 2*d^4*mu1^2*mu2^2*u^2*((mu2^2 - 1)*(u^2*(mu1^2 - 1) + d^2*mu1^2) - (mu2^2 - 1)*(d - d2)^2 - (mu1^2 - 1)*(d2 - v)^2 + d^2*mu2^2*(mu1^2 - 1)) - 4*d^4*mu1^2*mu2^2*(d - d2)^2*(d2 - v)^2 + 4*d^4*mu1^2*mu2^2*u*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1));

s10 = - 2*d^4*mu1^2*mu2^2*u^2*(2*d^2*mu1^2*u*(mu2^2 - 1) + 2*d^2*mu2^2*u*(mu1^2 - 1)) - 4*d^4*mu1^2*mu2^2*u*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1));

s11 = 4*d^8*mu1^4*mu2^4*u^2 + 2*d^4*mu1^2*mu2^2*u^2*(d^2*mu2^2*(u^2*(mu1^2 - 1) + d^2*mu1^2) - d^2*mu2^2*(d - d2)^2 - d^2*mu1^2*(d2 - v)^2 + d^2*mu1^2*u^2*(mu2^2 - 1));

s12 = (-4)*d^8*mu1^4*mu2^4*u^3;

s13 = d^8*mu1^4*mu2^4*u^4;



%[s1;s2;s3;s4;s5;s6;s7;s8;s9;s10;s11;s12;s13]


sol = roots([s1;s2;s3;s4;s5;s6;s7;s8;s9;s10;s11;s12;s13]);

idx = find(abs(imag(sol)) < 1e-6);
if(isempty(idx))
    disp('no solution');
    return
end

sol1 = sol(idx);
nn = size(sol1,1);


Normal = [0;-1];

for ii = 1:nn
    
    x = sol1(ii,1);
    vi = [x;d];
    
    v2 = RefractedRay(vi,Normal,1,mu1);
    q2 = vi + (d-d2)*v2/(v2'*Normal);
    
    
    v3 = RefractedRay(v2,Normal,mu1,mu2);
    
    vrd = [u;v] - q2;
    
    e = abs(vrd(1)*v3(2) - vrd(2)*v3(1));
    
    if(e < 1e-4)
        M = x*z2 + d*z1;
        return
    end
end



















