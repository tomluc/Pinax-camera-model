%
% Copyright (c) 2017 Jacobs University Robotics Group
% All rights reserved.
%
%
% Unless specified otherwise this code examples are released under 
% Creative Commons CC BY-NC-ND 4.0 license (free for non-commercial use). 
% Details may be found here: https://creativecommons.org/licenses/by-nc-nd/4.0/
%
%
% If you are interested in using this code commercially, 
% please contact us.
%
% THIS SOFTWARE IS PROVIDED BY Jacobs Robotics ``AS IS'' AND ANY
% EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL Jacobs Robotics BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% Contact: Tomasz Łuczyński: t.luczynski@jacobs-university.de / luczynski.tomek@gmail.com
%

clear;
clc;
%image info
width=1280;
height=960;

%example camera and lens parameters:
K=[1403.882943803675700, 0, 616.238499749930610;
    0, 1407.482157830063400, 479.792602354303430;
    0, 0, 1];
     
distCoeffs=[-0.273139587484678, 0.216093918966729, -0.000214253361360, -0.001770247078481, 0];


%setup parameters:
d1=10; %glass thickness
d0=1.4282; %physical d0 distance
d0virtual=[0;0;0.5851]; %virtual d0 distance
d2=d0+d1;

ng=1.5; %glass refraction index
nw=1.335; %water refraction index
mu_v=[ng;nw] ;
n=[0;0;1];

%image points
ImgPts=zeros(3,width*height);
N=size(ImgPts,2);
for i=1:width
    for j=1:height
        ImgPts(1,(j-1)*width+i)=i;
        ImgPts(2,(j-1)*width+i)=j;
        ImgPts(3,(j-1)*width+i)=1;
    end    
end

Rays=inv(K)*ImgPts;
M=zeros(3,N);

for i=1:N
    (i/N)*100
    p=5000*Rays(:,i)+d0virtual;    
    M(:,i) = SolveForwardProjectionCase3(d0,d2,-n,mu_v,p);
    
end

rvec=[0;0;0];
tvec=[0;0;0];

src2=zeros(1,N,3);
for itr=1:N
    src2(1,itr,1)=M(1,itr);
    src2(1,itr,2)=M(2,itr);
    src2(1,itr,3)=M(3,itr);
end

imagePoints = cv.projectPoints(src2, rvec, tvec, K, distCoeffs);

Mx=zeros(1,N);
My=zeros(1,N);

for i=1:N
    Mx(1,i)=imagePoints(i,1,1);
    My(1,i)=imagePoints(i,1,2);
end

save('XB3MapCenterX.txt','Mx','-ascii')
save('XB3MapCenterY.txt','My','-ascii')
%%
mapx=zeros(height,width);
mapy=zeros(height,width);

for i=1:width
    for j=1:height
        mapx(j,i)=Mx((j-1)*width+i);
        mapy(j,i)=My((j-1)*width+i);
    end    
end

img=cv.imread('testImg.jpg');
correctedImg=cv.remap(img,mapx,mapy);
cv.imwrite('remapped.jpg',correctedImg);







