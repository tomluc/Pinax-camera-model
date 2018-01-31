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
% Contact: robotics@jacobs-university.de

function [diff] = optim_d_0(x)

global dmax dmin K d1 ng nw dif

d0=x;
 
ImgPts=zeros(3,300);

for i=1:20
    for j=1:15
        ImgPts(1,(j-1)*20+i)=50*i;
        ImgPts(2,(j-1)*20+i)=50*j;
        ImgPts(3,(j-1)*20+i)=1;
    end
end

ZeroRays=inv(K)*ImgPts;

normal=[0;0;1];

t=[0;0;0];


dmin=9999;
dmax=-9999;

for i=1:300
    xl= RayTrace (ZeroRays(:,i), normal, ng, nw, d0, d1,t);
    xl=xl(:);
    po=[xl(10);xl(11);xl(12)];
    v2=[xl(13);xl(14);xl(15)];
    pom=abs(po(3)-v2(3)*(po(1)/(2*v2(1)) + po(2)/(2*v2(2))));
    dmin=min(dmin,pom);
    dmax=max(dmax,pom);
end
dif=dmax-dmin;
diff=dif;







