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


% Find absolute angular velocity associated with each link:
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

% Hip
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
% Actuated variables
qActuated = [q1;q2];

% D, C, G, and B matrices
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);


%%  Dynamics of Systems with Constraints
%Compute the Ground reaction Forces

% Compute the position of the stance foot (Leg 1) 
mu = 0.8; %define coefficient of friction
pst = [x - lL*cos(deg2rad(270) - (q1 + q3));...
       y - lL*sin(deg2rad(270) - (q1 + q3))];


% Compute the jacobian of the stance foot
JSt = jacobian(pst, q);


% Compute the time derivative of the Jacobian
dJSt = simplify(  jacobian(JSt*dq, q)  ) ;

% Compute the Stance Force
% [D   -JST';   [d2q ;     [-C*dq - G + B*u;
%  JSt   0]   *  FSt]   =   -dJSt * dq] ;
A = [D   -JSt';
     JSt  zeros(2)] ;
b = [-C*dq - G + B*u;
     -dJSt * dq] ;
d2q_FSt = A\b ;
FSt = simplify(  d2q_FSt(NDof+1:end)  ) ;

% Account for Friction
% If friction is too low, use these relations:
% split rows

Jx = JSt(1,:); Jz = JSt(2,:);
Jdotx = dJSt(1,:); Jdotz = dJSt(2,:);

% helper
H = C*dq + G;

% sticking solve (existing)
A = [D  -JSt'; JSt zeros(2)];
b = [-C*dq - G + B*u; -dJSt*dq];
sol = A\b;
lambda_stick = sol(NDof+1:end);    % [lambda_x; lambda_z]

% tangential slip velocity
vt = Jx * dq;                       % xdot_s

% Coulomb check
if abs(lambda_stick(1)) <= mu*lambda_stick(2)
    % sticking
    FSt = lambda_stick;
    ddq = sol(1:NDof);
else
    % sliding branch
    % regularize sign to avoid chattering
    eps_sign = 1e-8;
    signvt = sign(vt + eps_sign);

    % build scalar Schur term S = Jz * (D \ (Jz' - sign*mu*Jx'))
    S = Jz * (D \ (Jz' - signvt*mu*Jx'));

    % RHS for lambda_z: Jz * D^{-1} * (H - B*u) - Jdotz*dq
    rhs = Jz * (D \ (H - B*u)) - Jdotz * dq;

    % numerical safety: if S is near zero, fall back to augmented solve or regularize
    if abs(S) < 1e-9
        % fallback: use full augmented solve (treat as sticking) or regularize S
        % here we fall back to sticking solution
        FSt = lambda_stick;
        ddq = sol(1:NDof);
    else
        lambda_z = rhs / S;                     % scalar (or 1x1)
        lambda_x = -signvt * mu * lambda_z;
        FSt = [lambda_x; lambda_z];

        % compute accelerations
        ddq = D \ ( -C*dq - G + B*u + Jx'*lambda_x + Jz'*lambda_z );
    end
end





%% Impact Map

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


%% Other functions

% swing foot velocity
dpSw = JSw*dq;

%% Export functions
if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(FSt,      'File', 'gen/Fst_gen', 'Vars', {s, u});
matlabFunction(dqPlus,   'File', 'gen/dqPlus_gen', 'Vars', {s});
matlabFunction(pSw,      'File', 'gen/pSw_gen', 'Vars', {s});
matlabFunction(dpSw,     'File', 'gen/dpSw_gen', 'Vars', {s});
matlabFunction(pst,      'File', 'gen/pSt_gen', 'Vars', {s});
matlabFunction(pComLeg1, 'File', 'gen/pComLeg1_gen', 'Vars', {s});
matlabFunction(pComLeg2, 'File', 'gen/pComLeg2_gen', 'Vars', {s});
matlabFunction(pComTorso,'File', 'gen/pComTorso_gen', 'Vars', {s});
matlabFunction(pLeg1,    'File', 'gen/pLeg1_gen', 'Vars', {s});
matlabFunction(pLeg2,    'File', 'gen/pLeg2_gen', 'Vars', {s});




%% [Part 1a] Compute the f and g vectors
% Split FSt into 2 components: 1. which depends on u and 2. which does not depend on u
% Note: FSt is linear in u

Fst_u = jacobian(FSt, u) ; % FSt = Fst_u*u + (Fst - Fst_u*u)
Fst_u = simplify(Fst_u) ;

Fst_nu = FSt - Fst_u*u ; % Fst_nu = (Fst - Fst_u*u)
Fst_nu = simplify(Fst_nu) ;


% Compute vector fields
fvec = [dq ;
        inv(D)*(-C*dq - G + JSt'*Fst_nu)];
fvec = simplify(fvec);

gvec = [zeros(NDof, length(u));
          inv(D)*(B + JSt'*Fst_u)];
gvec = simplify(gvec);

% Write out files
matlabFunction(fvec, 'File', 'gen/fvec_gen', 'Vars', {s});
matlabFunction(gvec, 'File', 'gen/gvec_gen', 'Vars', {s});


%% Change of Coordinates
% Transformation matrix:
T = [1 0 0 0 0;
     0 1 0 0 0;
     0 0 1 0 1;
     0 0 0 1 1;
     0 0 0 0 1];
d = [0;
     0;
     -pi;
     -pi;
     0];

%% [Part 1b] Output dynamics
% Create absolute generalized coordinates
q_abs = T*q + d;

% Create symbolic variable needed for defining output
syms th3d real

% Define Output
y = [q_abs(2+3) - th3d;
     q_abs(2+2) + q_abs(2+1)];

% write out file
matlabFunction(y, 'File', 'gen/y_gen', 'Vars', {s, th3d});





%% [Part 1c] Lie Derivatives
% Define Lie Derivatives
Lfy = jacobian(y, s)*fvec;
Lf2y = jacobian(Lfy, s)*fvec;
LgLfy = jacobian(Lfy, s)*gvec;

% Write out files
matlabFunction(Lfy, 'File', 'gen/Lfy_gen', 'Vars', {s});
matlabFunction(Lf2y, 'File', 'gen/Lf2y_gen', 'Vars', {s});
matlabFunction(LgLfy, 'File', 'gen/LgLfy_gen', 'Vars', {s});






%% [Part 1d] Relabelling Matrix
params.R = [1, 0, 0, 0, 0;
            0, 1, 0, 0, 0;
            0, 0, 0, 1, 0;
            0, 0, 1, 0, 0;
            0, 0, 0, 0, 1];