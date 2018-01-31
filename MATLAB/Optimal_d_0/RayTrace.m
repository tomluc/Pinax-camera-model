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
%

function [refPts]=RayTrace (ray0, normal, mu, mu2, d0, d1,zero)

	v0=ray0/norm(ray0);
	normal=normal/norm(normal);
	pi=zero+ d0*v0/(v0'*normal);
    normal=-normal;
    c=-normal'*v0; 
    rglass=1/mu;
    rwater=1/mu2;
    v1=rglass*v0+(rglass*c -sqrt(1-rglass^2*(1-c^2)))*normal;
	v2=rwater*v0+(rwater*c -sqrt(1-rwater^2*(1-c^2)))*normal;
	normal=-normal;
    po=pi+ d1*v1/(v1'*normal);
	v1=v1/norm(v1);
    v2=v2/norm(v2);
	refPts=[v0;pi;v1;po;v2];
	
end