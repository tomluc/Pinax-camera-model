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

clear;
clc;

global dmax dmin K d1 ng nw dif

K=[900 0 512;
   0 900 384;
   0 0 1]; %camera intrinsics
d1=10; %glass thickness in mm
ng=1.492; %glass refraction index
nw=1.345; %water refraction index


x0=[0.1];
options = optimset('Display','iter','TolFun',1e-8,'TolX',1e-8,'MaxIter',100000,'MaxFunEvals',100000);
x = lsqnonlin(@optim_d_0,x0,[],[],options);
optimal_physical_d_0=x
virtual_d_0=(dmin+dmax)/2
approxErr=dif