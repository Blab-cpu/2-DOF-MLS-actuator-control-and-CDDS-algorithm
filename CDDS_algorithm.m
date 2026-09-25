%{

Author: Ragogna Riccardo
Year: March 2026

The code implements the CDDS tuning and simulation for the 2-Degree-of-Freedom
Magnetic Lead Screw actuator.

Procedure:
1) Compile the first section with your data-sets as described.
2) In section 2 put the mover rotation and linear position you want to simulate and implement the tuning 
3) Decide your desired model M(z)

Enjoy!
%}




%%

clearvars;
close all;
clear all;
clc;

Gr = 6.376954; % Lead-screw coefficient [mm]
%% 1. Input-Output Data collection

% %Insert your gains values

    Kp1 = 5.5;
    Kv1 = 0.089;

    Kp2 = 5.5;
    Kv2 = 0.089;

    Kp3 = 5.5;
    Kv3 = 0.089;

    Kp4 = 5.5;
    Kv4 = 0.089;

%Insert your data-set files and information
filename1 = 'Data-set1';
filename2 = 'Data-set2';
filename3 = 'Data-set3';
filename4 = 'Data-set4';


data1 = readmatrix(filename1);
data2 = readmatrix(filename2);
data3 = readmatrix(filename3);
data4 = readmatrix(filename4);

idx_start1 = 117857;
idx_end1   = idx_start1 + 50000;

idx_start2 = 91609; 
idx_end2   = idx_start2 + 50000;

idx_start3 = 115211; 
idx_end3   = idx_start3 + 50000;

idx_start4 = 120722; 
idx_end4   = idx_start4 + 50000;

step = 50;
Ts = 0.00002*step;

N2 = (idx_end1-idx_start1)/step;
N= N2+1;


% Data-set extraction
t_raw     = data1(idx_start1:step:idx_end1, 1);

theta_mis_R1 = data1(idx_start1:step:idx_end1, 4);  % Right rotor actual position from data-set1 
theta_mis_L1 = data1(idx_start1:step:idx_end1, 5);  % Left rotor actual position from data-set1
omega_mis_R1 = data1(idx_start1:step:idx_end1, 2);  % Right rotor actual speed from data-set1
omega_mis_L1 = data1(idx_start1:step:idx_end1, 3);  % Left rotor actual position from data-set1


theta_mis_R2 = data2(idx_start2:step:idx_end2, 4);  % Right rotor actual position from data-set2
theta_mis_L2 = data2(idx_start2:step:idx_end2, 5);  
omega_mis_R2 = data2(idx_start2:step:idx_end2, 2);  
omega_mis_L2 = data2(idx_start2:step:idx_end2, 3);  


theta_mis_R3 = data3(idx_start3:step:idx_end3, 4);  
theta_mis_L3 = data3(idx_start3:step:idx_end3, 5);  
omega_mis_R3 = data3(idx_start3:step:idx_end3, 2);  
omega_mis_L3 = data3(idx_start3:step:idx_end3, 3);  

theta_mis_R4 = data4(idx_start4:step:idx_end4, 4);  
theta_mis_L4 = data4(idx_start4:step:idx_end4, 5);  
omega_mis_R4 = data4(idx_start4:step:idx_end4, 2);  
omega_mis_L4 = data4(idx_start4:step:idx_end4, 3); 

% WARNING: the following variables can be measured while in this
% implementation it was not possible caused by external software issues.

% Put here the references used for the data-sets generation
ref_R1   =  -5.5708 * ones(1, N)';
ref_L1   =  2.4292 * ones(1, N)';

omega_ref_R1 = ref_R1 - theta_mis_R1;
omega_ref_L1 = ref_L1 - theta_mis_L1;

ref_R2   =  5.7854 * ones(1, N)';
ref_L2   = -4.2146 * ones(1, N)';

omega_ref_R2 = ref_R2 - theta_mis_R2;
omega_ref_L2 = ref_L2 - theta_mis_L2;

ref_R3   =  7.5708 * ones(1, N)';
ref_L3   = -4.4292 * ones(1, N)';

omega_ref_R3 = ref_R3 - theta_mis_R3;
omega_ref_L3 = ref_L3 - theta_mis_L3;

ref_R4   =  -2.42926 * ones(1, N)';
ref_L4   =  5.5708 * ones(1, N)';


% Put here the current references from the data-sets (here generated not measured)
iq_mis_R1 = Kp1*Kv1*(ref_R1 - theta_mis_R1) - Kv1*(omega_mis_R1);
iq_mis_L1 = Kp1*Kv1*(ref_L1 - theta_mis_L1) - Kv1*(omega_mis_L1);

iq_mis_R2 = Kp2*Kv2*(ref_R2 - theta_mis_R2) - Kv2*(omega_mis_R2);
iq_mis_L2 = Kp2*Kv2*(ref_L2 - theta_mis_L2) - Kv2*(omega_mis_L2);

iq_mis_R3 = Kp3*Kv3*(ref_R3 - theta_mis_R3) - Kv3*(omega_mis_R3);
iq_mis_L3 = Kp3*Kv3*(ref_L3 - theta_mis_L3) - Kv3*(omega_mis_L3);

iq_mis_R4 = Kp4*Kv4*(ref_R4 - theta_mis_R4) - Kv4*(omega_mis_R4);
iq_mis_L4 = Kp4*Kv4*(ref_L4 - theta_mis_L4) - Kv4*(omega_mis_L4);

r1 = [ref_R1 ref_L1];
r2 = [ref_R2 ref_L2];
r3 = [ref_R3 ref_L3];
r4 = [ref_R4 ref_L4];

%% 2. Reference setup

% Put your rotary and linear position reference you want to tune/simulate
thetaM_ref = 4; 
xM_ref = 10;


ref_R   =  (-thetaM_ref -  xM_ref*Gr^-1)*ones(1, N)';
ref_L   =   (thetaM_ref -  xM_ref*Gr^-1)*ones(1, N)';
r = [ref_R ref_L];


%% 3. Desired model setup

Trise= 0.2; %[s]
wn=3.35/Trise; %[rad/s]
epsilon = 1;


s = tf('s');        % Laplace operator s
z = tf('z', Ts);    % Z-operator z


M = c2d([wn^2/(s^2 + s*2*epsilon*wn+wn^2) , 0; s*wn^2/(s^2 + s*2*epsilon*wn+wn^2) ,  0; 0 , wn^2/(s^2 + s*2*epsilon*wn+wn^2);0 , s*wn^2/(s^2 + s*2*epsilon*wn+wn^2)], Ts).*[z^-1  0;  z^-1 0;0 z^-1 ;0  z^-1]; %RITARDO??

% Desired evolution 
yod = lsim(M, r)';   % 4xN


t = (1:length(yod))*Ts; % Time vector






%% Tuning

yod_R = [yod(1,:); yod(2,:)]; %2xN

yod_L = [yod(3,:); yod(4,:)]; %2xN


xd = [ref_R'; ref_L';  yod(1,:) ; yod(3,:)];    %1xN

yodd = [ yod(1,:); yod(3,:)];

uho01 = [1 0 -1 0; 0 0 0 0 ] * xd;    %1xN
uho02 = lsim([0 0 -(z-1)/(Ts*z) 0 ; 0 0 0 0 ], xd)';% with eulero
uho03 = [0 0 0 0; 0 1 0 -1 ] * xd;
uho04 = lsim([0 0 0 0 ; 0 0 0 -(z-1)/(Ts*z) ], xd)'; % with eulero


yho01 = zeros(2,N);        
yho02 = zeros(2,N);        
yho03 = zeros(2,N);        
yho04 = zeros(2,N);        

v1 = zeros(4,N);       % Linked to position error right
v2 = zeros(4,N);       % Linked to speed error right
v3 = zeros(4,N);       % Linked to position error left 
v4 = zeros(4,N);       % Linked to speed error left

y1 = [theta_mis_R1'; theta_mis_L1'];
y2 = [theta_mis_R2'; theta_mis_L2']; 
y3 = [theta_mis_R3'; theta_mis_L3'];
y4 = [theta_mis_R4'; theta_mis_L4'];

u1=[iq_mis_R1' ; iq_mis_L1']; % contain the first experiment [iqR iqL] as 2xN
u2=[iq_mis_R2' ; iq_mis_L2']; % contain the second experiment [iqR iqL] as 2xN
u3=[iq_mis_R3' ; iq_mis_L3']; % contain the third experiment [iqR iqL] as 2xN
u4=[iq_mis_R4' ; iq_mis_L4']; % contain the 4th experiment [iqR iqL] as 2xN


ut = [u1(:,1) u2(:,1) u3(:,1) u4(:,1)];
uti = ut' / (ut * ut'); % Monrooe inverse

for k = 1:N
    tmp1 = uho01(:,k);
    tmp2 = uho02(:,k);
     tmp3 = uho03(:,k);
      tmp4 = uho04(:,k);
    for i = 1:(k-1)
        tmp1 = tmp1 - [u1(:,k-i+1) u2(:,k-i+1) u3(:,k-i+1) u4(:,k-i+1)] * v1(:,i);
        tmp2 = tmp2 - [u1(:,k-i+1) u2(:,k-i+1) u3(:,k-i+1) u4(:,k-i+1)] * v2(:,i);
        tmp3 = tmp3 - [u1(:,k-i+1) u2(:,k-i+1) u3(:,k-i+1) u4(:,k-i+1)] * v3(:,i);
        tmp4 = tmp4 - [u1(:,k-i+1) u2(:,k-i+1) u3(:,k-i+1) u4(:,k-i+1)] * v4(:,i);

    end
    v1(:,k) = uti* tmp1;
    v2(:,k) = uti* tmp2;
    v3(:,k) = uti* tmp3;
    v4(:,k) = uti* tmp4;
    
    for i = 1:k
        yho01(:,k) = yho01(:,k) + [y1(:,k-i+1)  y2(:,k-i+1) y3(:,k-i+1) y4(:,k-i+1)] * v1(:,i);
        yho02(:,k) = yho02(:,k) + [y1(:,k-i+1)  y2(:,k-i+1) y3(:,k-i+1) y4(:,k-i+1)] * v2(:,i);
        yho03(:,k) = yho03(:,k) + [y1(:,k-i+1)  y2(:,k-i+1) y3(:,k-i+1) y4(:,k-i+1)] * v3(:,i);
        yho04(:,k) = yho04(:,k) + [y1(:,k-i+1)  y2(:,k-i+1) y3(:,k-i+1) y4(:,k-i+1)] * v4(:,i);
       
    end
end

%Weight matrix
W = [1 0; 0 1];
A = zeros(4,4);
b = zeros(4,1);
% yod = psi1* yho01 + psi2* yho02


for k = 1:N2


    yho0 = [yho01(:,k) yho02(:,k) yho03(:,k) yho04(:,k)];

    A = A + yho0' * W * yho0;
    b = b + yho0' * W * yodd(:,k);
end
th = pinv(A) * b;
%th'

Kv_R = th(2)  
Kp_R = th(1)/Kv_R

Kv_L = th(4)  
Kp_L = th(3)/Kv_L






y_re_built = th(1)*yho01 + th(2)*yho02 + th(3)*yho03 + th(4)*yho04;
  
 





%% Simulation

alpha_filter = 1;  % Sometimes the simulation can diverge, put the alpha_filter to 0.999999..

uh = zeros(2,N);
yh = zeros(2,N);

r=r';

v = zeros(4,N);       



for k = 1:N


    if (k > 1)
       

if k > 2
         
        raw_derivative = (yh(:,k-1) - yh(:,k-2)) / Ts;

        derivative = (1 - alpha_filter) * derivative_prev + alpha_filter * raw_derivative;
    else
        derivative = [0; 0];
end

derivative_prev = derivative;


        term1 = Kp_R * Kv_R * (r(:,k-1) - yh(:,k-1)); 


        term2 = - Kv_R * derivative;                


        uh(:,k) = term1 + term2; 
    else
        uh(:,k) = [0;0];
    end


    tmp = uh(:,k); 
    for i = 1:(k-1)
        

        tmp = tmp - [u1(:,k-i+1) u2(:,k-i+1) u3(:,k-i+1) u4(:,k-i+1)] * v(:,i);
    end
    v(:,k) = uti * tmp;

    
    yh(:,k) = [0;0]; 
    for i = 1:k
        yh(:,k) = yh(:,k) + [y1(:,k-i+1) y2(:,k-i+1) y3(:,k-i+1) y4(:,k-i+1)] * v(:,i);     
    end
end


%% Simulated - Actual output ERRORS for general test 

error_Angle_R_simulation = yod_R (1,:) - yh(1,:); 

rmse_angle_R_simulation = sqrt(mean(error_Angle_R_simulation.^2))

norm_err_angle_R_simulation = norm(error_Angle_R_simulation);
norm_ref_angle_R_simulation = norm(yod_R(1,:)-mean(yod_R(1,:)));
FIT_angle_R_simulation = 100*(1-(norm_err_angle_R_simulation/norm_ref_angle_R_simulation)) 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

error_Angle_L_simulation = yod_L (1,:) - yh(2,:); 

rmse_angle_L_simulation = sqrt(mean(error_Angle_L_simulation.^2))

norm_err_angle_L_simulation = norm(error_Angle_L_simulation);
norm_ref_angle_L_simulation = norm(yod_L(1,:)-mean(yod_L(1,:)));
FIT_angle_L_simulation = 100*(1-(norm_err_angle_L_simulation/norm_ref_angle_L_simulation)) 

