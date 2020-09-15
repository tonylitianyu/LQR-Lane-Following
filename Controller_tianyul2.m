function func = Controller
% INTERFACE
%
%   sensors
%       .e_lateral      (error in lateral position relative to road)
%       .e_heading      (error in heading relative to road)
%       .v              (forward speed)
%       .w              (turning rate)
%       .r_road         (signed radius of curvature of road - to find the
%                        corresponding turning rate for a given forward
%                        speed: w_road = v_road/sensors.r_road)
%
%   references
%       (none)
%
%   parameters
%       .tStep      (time step)
%       .tauMax     (maximum wheel torque)
%       .roadwidth  (width of road - half on each side of centerline)
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .tauR       (right wheel torque)
%       .tauL       (left wheel torque)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)
%obtain equations
load('DesignProblem04_EOMs.mat');
f = symEOM.f;
syms phi phidot v w tauR tauL e_lateral e_heading v_road w_road real

%define state
b = [phi; phidot; v; w; e_lateral; e_heading];

b_e = [0; 0; 3; 0; 0; 0];

u = [tauR; tauL];
v_road = 1;
r_road = sensors.r_road;
w_road = v_road/r_road;
edot_lateral = -v*sin(e_heading);
edot_heading = w-(((v*cos(e_heading))/(v_road+w_road*e_lateral))*w_road);

%calculate A
a = [phidot; f; edot_lateral; edot_heading];
Aj = jacobian(a,b);
A = double(vpa(subs(Aj,b,b_e),4));

%calculate B
Bj = jacobian(a,u);
B = double(vpa(subs(Bj,[b;u] ,[b_e; 0; 0]),4));

%define C
C = [zeros(4,2) eye(4)];


format shortG
%calculate W controllability
Wc = ctrb(A,B);
rank(Wc);

%calculate eigenvalue for A
[V D] = eig(A);


% load('tune.mat')
%find K
Q_c = diag([1 1 3 1 1000 1000]);
R_c = eye(2);
K = lqr(A,B,Q_c,R_c);
[VC,DC] = eig(A-B*K);

%calculate observability
Wo = obsv(A,C);
rank(Wo);

%calculate L
Q_o = diag([1 1 1 1]);
R_o = diag([1 1 1 1 0.001 0.001]);
L = lqr(A',C',inv(R_o),inv(Q_o))';
[VO,DO] = eig(A-L*C);


% %get data from calculation
data.xhat = [0;0;-b_e(3);0;0;0];
data.A = A;
data.B = B;
data.C = C;
data.K = K;
data.L = L;
data.eq = b_e;
data.y = [sensors.v-data.eq(3); sensors.w-data.eq(4); sensors.e_lateral-data.eq(5); sensors.e_heading-data.eq(6)];

[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)

h = parameters.tStep; 
u = -data.K*data.xhat;
data.y = [sensors.v-data.eq(3); sensors.w-data.eq(4); sensors.e_lateral-data.eq(5); sensors.e_heading-data.eq(6)];
%update xhat
data.xhat = data.xhat+(data.A*data.xhat+data.B*(-data.K*data.xhat)-...
    data.L*(data.C*data.xhat-data.y))*h;

%input
actuators.tauR = u(1);
actuators.tauL = u(2);
end