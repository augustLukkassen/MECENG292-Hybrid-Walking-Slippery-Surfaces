% ME193B/293B Feedback Control of Legged Robots
% UC Berkeley
clear
%% Define symbolic variables for cofiguration variables and mechanical parameters

syms q1 q2 q3 x y real
syms dq1 dq2 dq3 dx dy real
syms u1 u2 real

% Position Variable vector
q = [x;y;q1;q2;q3];

% Velocity variable vector
dq = [dx;dy;dq1;dq2;dq3];

% State vector
s = [q;dq];

% Inputs
u = [u1;u2];

% parameters
          
lL = 1;
lT = 0.5;
mL = 5;
mT = 10;
JL = 0;
JT = 0;
mH = 15;
g = 9.81;

% # of Degrees of freedom
NDof = length(q);


%% Problem 1: Lagrangian Dynamics
%Find the CoM position of each link

% Torso
pComTorso = [x + lT*sin(q3);...
             y + lT*cos(q3)];

% Leg 1
pComLeg1 = [x - lL/2*cos(deg2rad(270) - (q1 + q3));...
            y - lL/2*sin(deg2rad(270) - (q1 + q3))];

% Leg 2
pComLeg2 = [x + lL/2*cos(q2 + q3 - deg2rad(90));...
            y - lL/2*sin(q2 + q3 - deg2rad(90))];


% Leg 1
pLeg1 = [x - lL*cos(deg2rad(270) - (q1 + q3));...
            y - lL*sin(deg2rad(270) - (q1 + q3))];

% Leg 2
pLeg2 = [x + lL*cos(q2 + q3 - deg2rad(90));...
            y - lL*sin(q2 + q3 - deg2rad(90))];

% Find the CoM velocity of each link

% Torso
dpComTorso = simplify(jacobian(pComTorso, q)*dq);

% Leg 1
dpComLeg1 = simplify(jacobian(pComLeg1, q)*dq);

% Leg 2
dpComLeg2 = simplify(jacobian(pComLeg2, q)*dq);


%% Find absolute angular velocity associated with each link:
% Torso
dq3Absolute = dq3;
% Leg 1
dq1Absolute = dq3 + dq1;
% Leg 2
dq2Absolute = dq3 + dq2;

% Total Kinetic energy = Sum of kinetic energy of each link

% Torso
KETorso = 0.5*mT*dpComTorso(1)^2 + 0.5*mT*dpComTorso(2)^2 + 0.5*JT*dq3Absolute^2;

% Leg 1
KELeg1 = 0.5*mL*dpComLeg1(1)^2 + 0.5*mL*dpComLeg1(2)^2 + 0.5*JL*dq1Absolute^2;

% Leg 2
KELeg2 = 0.5*mL*dpComLeg2(1)^2 + 0.5*mL*dpComLeg2(2)^2 + 0.5*JL*dq2Absolute^2;

KEHip = 0.5*mH*dx^2 + 0.5*mH*dy^2;
% Total KE
KE = simplify(KETorso + KELeg1 + KELeg2 + KEHip);

% Total potential energy = Sum of Potential energy of each link

% Torso
PETorso = mT*g*pComTorso(2);

%Leg 1
PELeg1 = mL*g*pComLeg1(2);

% Leg 2
PELeg2 = mL*g*pComLeg2(2);

% Hip
PEHip = mH*g*y;
% Total PE
PE = simplify(PETorso + PELeg1 + PELeg2 + PEHip);

% Lagrangian

L = KE - PE;

% Equations of Motion
EOM = jacobian(jacobian(L,dq), q)*dq - jacobian(L, q)' ;
EOM = simplify(EOM);

% Find the D, C, G, and B matrices

% Actuated variables
qActuated = [q1;q2];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);


%%  Dynamics of Systems with Constraints

% Stick 
%Compute the Ground reaction Forces

% Compute the position of the stance foot (Leg 1) 
pst = [x - lL*cos(deg2rad(270) - (q1 + q3));...
       y - lL*sin(deg2rad(270) - (q1 + q3))];


% Compute the jacobian of the stance foot
JSt = jacobian(pst, q);

% Compute the time derivative of the Jacobian
dJSt = sym(zeros(size(JSt)));
for i = 1:size(JSt, 1)
    for j = 1:size(JSt, 2)
        dJSt(i, j) = simplify(jacobian(JSt(i, j), q)*dq);
    end
end

H = C*dq + G;
alpha = 0;
% Constraint Force to enforce the holonomic constraint:
FSt = - pinv(JSt*(D\JSt'))*(JSt*(D\(-H + B*u)) + dJSt*dq + 2*alpha*JSt*dq + alpha^2*pst);
FSt = simplify(FSt);

% Split FSt into 2 components: 
%   1. which depends on u and 
%   2. which does not depend on u 
% Note: FSt is linear in u

Fst_u = jacobian(FSt, u); % FSt = Fst_u*u + (Fst - Fst_u*u)
Fst_nu = FSt - Fst_u*u; % Fst_nu = (Fst - Fst_u*u)



% Slip 

J_n = JSt(2, :) ; % Horizontal 

dJSt_n = sym(zeros(size(J_n)));
for j = 1:size(J_n, 2)
    dJSt_n(1, j) = simplify(jacobian(J_n(1, j), q) * dq);
end

% Normal Force 
F_n = - pinv(J_n*(D\J_n')) * ( J_n*(D\(-H + B*u)) + dJSt_n*dq);

% Tangential Force 
mu_s = 0.4 ;

J_t = JSt(1, :) ; 
dx_s = J_t * dq ; 
F_t = -mu_s * F_n * sign(dx_s) ;

FSt_slip = [F_t ; F_n] ;

Fst_u_slip = jacobian(FSt_slip, u) ; 
Fst_nu_slip = FSt_slip - Fst_u_slip*u ;


%% Impact Map
% Stick 

% Compute the swing leg position (leg 2)
pSw = [x + lL*cos(q2 + q3 - deg2rad(90));...
       y - lL*sin(q2 + q3 - deg2rad(90))];

JSw = jacobian(pSw, q);

% postImpact = [qPlus;F_impact];
% Here, q, dq represent the pre-impact positions and velocities
[postImpact] = ([D, -JSw';JSw, zeros(2)])\[D*dq;zeros(2,1)];

% Post Impact velocities
dqPlus = simplify(postImpact(1:NDof));

% Impact Force Magnitude
Fimpact = simplify(postImpact(NDof+1:NDof+2));

% Slip 
J_x = JSw(1, :) ;
J_z = JSw(2, :) ;
J_thilde = J_z - sign(J_x*dq)*mu_s*J_x ; 

dqPlus_s = (eye(NDof)-(D\J_thilde.')*inv(J_z*(D\J_thilde.'))*J_z)*dq ; 


%% Other functions

% swing foot velocity
dpSw = JSw*dq;

% stance foot velocity 
dpSt = JSt*dq ; 


%% Export functions
if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(FSt, 'File', 'gen/Fst_gen', 'Vars', {s, u});
matlabFunction(FSt_slip, 'File', 'gen/Fst_s_gen', 'Vars', {s, u});
matlabFunction(Fimpact, 'File', 'gen/Fimpact_gen', 'Vars', {s});
matlabFunction(dqPlus, 'File', 'gen/dqPlus_gen', 'Vars', {s});
matlabFunction(dqPlus_s, 'File', 'gen/dqPlus_s_gen', 'Vars', {s});
matlabFunction(pSw, 'File', 'gen/pSw_gen', 'Vars', {s});
matlabFunction(dpSw, 'File', 'gen/dpSw_gen', 'Vars', {s});
matlabFunction(dpSt, 'File', 'gen/dpSt_gen', 'Vars', {s});
matlabFunction(pst, 'File', 'gen/pSt_gen', 'Vars', {s});
matlabFunction(pComLeg1, 'File', 'gen/pComLeg1_gen', 'Vars', {s});
matlabFunction(pComLeg2, 'File', 'gen/pComLeg2_gen', 'Vars', {s});
matlabFunction(pComTorso, 'File', 'gen/pComTorso_gen', 'Vars', {s});
matlabFunction(pLeg1, 'File', 'gen/pLeg1_gen', 'Vars', {s});
matlabFunction(pLeg2, 'File', 'gen/pLeg2_gen', 'Vars', {s});


%% [Part 1a] Compute the f and g vectors

% Stick 
f0 = [dq; 
      D\(-C*dq - G + JSt'*Fst_nu)] ; 

g0 = [zeros(5,2); 
      D\(B + JSt'*Fst_u)] ; 

matlabFunction(f0, 'File', 'gen/f_gen', 'Vars', {s}, 'Optimize', false);
matlabFunction(g0, 'File', 'gen/g_gen', 'Vars', {s}, 'Optimize', false);

% Slip 
fs = [dq; 
      D\(-C*dq - G + JSt'*Fst_nu_slip)] ; 

gs = [zeros(5,2); 
      D\(B + JSt'*Fst_u_slip)] ; 

matlabFunction(fs, 'File', 'gen/fs_gen', 'Vars', {s}, 'Optimize', false);
matlabFunction(gs, 'File', 'gen/gs_gen', 'Vars', {s}, 'Optimize', false);

%% Change of Coordinates

T = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 1;
     0 0 0 1 1;
     0 0 0 0 1] ;

d = [0;
     0;
     -pi;
     -pi;
     0] ;


%% [Part 1b] Output dynamics
syms a0 a1 a2 a3 b0 b1 b2 b3 real
th1d = pi/8 ; 

q_abs = T * q + d  ;
th1 = q_abs(3) ; 
th2 = q_abs(4) ;
th3 = q_abs(5) ; 

h_d1 = a0 + a1*th1 + a2*th1^2 + a3*th1^3 ; 
h_d2 = -th1 + (b0 + b1*th1 + b2*th1^2 + b3*th1^3)*(th1 + th1d)*(th1 - th1d) ;
y = [th3 - h_d1; 
     th2 - h_d2] ;


%% [Part 1c] Lie Derivatives

% Stick 
Lfy = jacobian(y, s) * f0 ;
Lf2y = jacobian(Lfy, s) * f0 ; 
LgLfy = jacobian(Lfy,s) * g0; 

matlabFunction(y, Lfy, Lf2y, LgLfy, 'File', 'gen/lie_derivatives_gen', 'Vars', ...
    {s, a0, a1, a2, a3, b0, b1, b2, b3}, 'Optimize', false);

% Slip
Lfy_s = jacobian(y, s) * fs ;
Lf2y_s = jacobian(Lfy_s, s) * fs ; 
LgLfy_s = jacobian(Lfy_s,s) * gs; 

matlabFunction(y, Lfy_s, Lf2y_s, LgLfy_s, 'File', 'gen/lie_derivatives_gen_s', 'Vars', ...
    {s, a0, a1, a2, a3, b0, b1, b2, b3}, 'Optimize', false);
%% [Part 1d] Relabelling Matrix
R = eye(5) ; 
R(3:4,3:4) = [0 1; 1 0] ;