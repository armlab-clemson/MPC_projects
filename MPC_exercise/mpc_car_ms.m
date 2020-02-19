
addpath('C:\Users\srivats\Documents\MATLAB\casadi-v3.5.1')
import casadi.*


%% Data
T = 0.1; % sampling time [s]
N = 150; % prediction horizon
L = 2.7;

v_max=15; v_min=-0.25*v_max;  
delta_max= 1.4; delta_min= -delta_max;
%% Problem Setup
x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');
states = [x;y;psi]; n_states = length(states);

v = SX.sym('v'); delta = SX.sym('delta');
controls = [v;delta]; n_controls = length(controls);

%% f(x,u)
rhs = [v*cos(psi);v*sin(psi);(v/L)*tan(delta)]; 
f = Function('f',{states,controls},{rhs}); 

%% MPC setup
U = SX.sym('U',n_controls,N); 
P = SX.sym('P',n_states + n_states);
X = SX.sym('X',n_states,(N+1));
obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

%% System constraints

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -5; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 25; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -5; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 25; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state psi lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state psi upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = delta_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = delta_max; %omega upper bound
%% Simulation

t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);  % two control inputs 
X0= repmat(x0,1,N+1)';
sim_tim = 100; % Maximum simulation time

%% MPC Loop
mpciter = 0;
xx1 = [];
u_cl=[];

main_loop = tic;
while(norm((x0-xs),2) > 9e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)]; % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)';
    
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)';
    
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = shift(T, t0, x0, u,f); % get the initialization of the next optimization step
    
    xx(:,mpciter+2) = x0;  
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2)
average_mpc_time = main_loop_time/(mpciter+1)

if xs(3)==pi/2 || xs(3)==-pi/2
   plot_car2(xx,xx1,xs,N,L,t,u_cl);
else
   plot_car (xx,xx1,xs,N,L,t,u_cl);
end