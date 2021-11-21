% MATLAB code for computing and simulating Linear Quadratic Regulator (LQR)
% for nonzero set points 
% Author: Aleksandar Haber, November 20, 2021, Rochester, NY

clear, pack, clc

% mass
m=10 
% damping 
kd=5
% spring constant
ks=10


%% state-space system representation 

A=[0 1; -ks/m -kd/m];
B=[0; 1/m];
C=[1 0];
D=0;
sysSS=ss(A,B,C,D)

%% define the desired state 
xd=[3;0];


%% compute ud

ud=-inv(B'*B)*B'*A*xd


%% simulate the system response for the computed ud

% set initial condition
x0=1*randn(2,1);

%final simulation time 

tFinal=30;

time_total=0:0.1:tFinal;
input_ud=ud*ones(size(time_total));

[output_ud_only,time_ud_only,state_ud_only] = lsim(sysSS,input_ud,time_total,x0); 

figure(1)
plot(time_total,state_ud_only(:,1),'k')
hold on 
plot(time_total,state_ud_only(:,2),'r')
xlabel('Time [s]')
ylabel('Position and velocity')
legend('Position','Velocity')
grid

%% simulate the LQR algorithm 

Q= 10*eye(2,2);
R= 0.1;
N= zeros(2,1);

[K,S,e] = lqr(sysSS,Q,R,N);

% compute the eigenvalues of the closed loop system 
eig(A-B*K)

% eigenvalues of the open-loop system 
eig(A)

%simulate the closed loop system 
% closed loop matrix 
Acl= A-B*K;

Bcl=-Acl;
C=[1 0];
D=0;
sysSSClosedLoop=ss(Acl,Bcl,C,D)

closed_loop_input=[xd(1)*ones(size(time_total));
                    xd(2)*ones(size(time_total))];


[output_closed_loop,time_closed_loop,state_closed_loop] = lsim(sysSSClosedLoop,closed_loop_input,time_total,x0); 




figure(2)
plot(time_total,state_ud_only(:,1),'k')
hold on 
plot(time_total,state_ud_only(:,2),'r')
xlabel('Time [s]')
ylabel('Position and velocity')
plot(time_closed_loop,state_closed_loop(:,1),'m')
hold on 
plot(time_closed_loop,state_closed_loop(:,2),'b')
legend('Position-Constant Input ','Velocity-Constant Input','Position-Closed Loop ','Velocity-Closed Loop')
grid








