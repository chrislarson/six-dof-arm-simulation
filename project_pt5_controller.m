clear;clc;

% Open-loop system model
A = [0, 1; 0, 0];
B = [0; 1]; 
C = [1 0];
D = 0;
ol_poles = eig(A);
ol_sys = ss(A,B,C,D);
% ol_step_info = stepinfo(ol_sys);

% % Add system error 
Ar = A;
% Ar(end,:) = Ar(end,:) + 2*(rand([3; 1])).*Ar(end,:)*.1;

trace_times = [1.5, 2, 2.5, 3, 3.5, 4];
k_gains = zeros(6, 2);
dxs = zeros(6,1);
rise_times = zeros(6,1);
settling_times = zeros(6,1);
sigmas = zeros(6,1);
wns = zeros(6,1);
zetas = zeros(6,1);
GMs = zeros(6,1);
PMs = zeros(6,1);

N = length(trace_times);
for i = 1:N
    % Define intended system behavior
    dx = trace_times(:,i) / 25.0;
    dxs(i) = dx;
    Ts = dx * 0.75;
    Tr = Ts * 0.2;
    os = 0.05;

    settling_times(i) = Ts;
    rise_times(i) = Tr;

    % Continuous time design - compute poles
    zeta = sqrt(log(os)^2 / (pi^2 + log(os)^2));
    zetas(i) = zeta;
    wn_ts = 4.6/(zeta * Ts);
    wn_tr = (1.53 + 2.31*zeta^2) / Tr;
    wn = max(wn_ts, wn_tr);
    wns(i) = wn;
    sigma = zeta*wn;
    sigmas(i) = sigma;
    
    % Continuous time design - compute poles
    cl_poles = [
        -wn*(zeta+sqrt(1-zeta^2)*1j);
        -wn*(zeta-sqrt(1-zeta^2)*1j);
    ];
    K = place(A,B,cl_poles);
    
    % Closed-loop system model - continuous time
    A_cl = A-B*K;
    B_cl = B*(K-(B'*B)^-1*B'*A)*[1;0];
    cl_sys = ss(A_cl, B_cl, C, D);
    cl_step_info = stepinfo(cl_sys);
    
    % Closed-loop system plots - continuous time
    % figure(2);
    % hold on;
    % subplot(2,1,1);
    % step(cl_sys);
    % subplot(2,1,2);
    % margin(cl_sys);
    % hold off;
    
    % Closed-loop system root locus plot
    % figure(10);
    % title("Root Locus, Closed-loop System")
    % hold on;
    % rlocus(cl_sys);
    % sgrid(zeta,wn)
    % hold off;
    
    % Discrete time design
    dt = 0.001;
    Ad = expm(A*dt);
    Bd = A'*(Ad-eye(size(A)))*B;
    cl_poles_disc = exp(cl_poles*dt);
    Kd = place(Ad, Bd, cl_poles_disc);
    k_gains(i,:) = Kd;
    
    % Closed-loop system model - discrete time
    Ad_cl = Ad-Bd*Kd;
    Bd_cl = Bd*(Kd-(B'*B)^-1*B'*A)*[1;0];
    dcl_sys = ss(Ad_cl, Bd_cl, C, D, dt);
    dcl_step_info = stepinfo(dcl_sys);

    [Gm,Pm,Wgm,Wpm] = margin(dcl_sys);
    GMs(i) = Gm;
    PMs(i) = Pm;
    
    % Closed-loop system plots - discrete time
    % figure(3);
    % hold on;
    % subplot(2,1,1);
    % step(dcl_sys);
    % subplot(2,1,2);
    % margin(dcl_sys);
    % hold off;

end

trace_times 
dxs
rise_times
settling_times
zetas
wns 
k_gains
k_gains(:,1)
k_gains(:,2)



