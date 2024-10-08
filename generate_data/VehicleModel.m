function [ExactMeasurements, DifferentialStates] = VehicleModel(DeltaWheel_rad, DriveForce_act_N, extForces_N, extTorques_Nm, States, vp)

% differential equations for the nonlinear single track model

%% parameter mapping
tw_front_m = vp.tw_front_m;  
tw_rear_m = vp.tw_rear_m;   
l_front_m = vp.l_front_m;   
l_rear_m = vp.l_total_m - vp.l_front_m;      
m = vp.m_Vehicle_kg;                
J = vp.VehicleInertia_kgm2;         

cD = vp.cDrag;                        
roh = 1.22;                         
A_ref = vp.Aref_m2;                 

cF = 0.1*vp.cRollFriction_Npmps;       
cW_F = -1 * vp.cLiftF; 
cW_R = -1 * vp.cLiftR; 

PacFrontLat = vp.PacLatF;           % estimate
PacRearLat = vp.PacLatR;            % estimate
PacFrontLong = vp.PacLongF;         % estimate
PacRearLong = vp.PacLongR;          % estimate

tyreradius_front_m = vp.r_tireF_m;  
tyreradius_rear_m = vp.r_tireR_m; 
WheelInertia_Front_kgm2 = vp.WheelInertiaF_kgm2; 
WheelInertia_Rear_kgm2 = vp.WheelInertiaR_kgm2; 

% initialize outputs
DifferentialStates = zeros(15, 1); 
ExactMeasurements = zeros(9, 1); 

% get states 
vx_mps = States(1); 
vy_mps = States(2);
dPsi_rad = States(3); 
omega_rad = States(4:7); 
% lambda_perc = States(8:11); 
% alpha_rad = States(12:15); 

% powertrain bias front rear
FxPT_N = DriveForce_act_N; 

% wheel inertia factor matrix 
wheelInertia_factors = [tyreradius_front_m/WheelInertia_Front_kgm2;...
  tyreradius_front_m/WheelInertia_Front_kgm2;...
  tyreradius_rear_m/WheelInertia_Rear_kgm2;...
  tyreradius_rear_m/WheelInertia_Rear_kgm2];

%% calculate friction, aero and tire forces 
FxFriction = cF*vx_mps + 0.5*cD*roh*A_ref*vx_mps^2;
Fz_N = [ones(2, 1).*((m*9.81+extForces_N(3))*l_rear_m/(l_rear_m+l_front_m)/2 + 0.5*0.5*roh*cW_F*A_ref*vx_mps.^2);...
  ones(2, 1).*((m*9.81+extForces_N(3))*l_front_m/(l_rear_m+l_front_m)/2 + 0.5*0.5*roh*cW_R*A_ref*vx_mps.^2)]; 

% calculate slip updates
[lambda_perc, alpha_rad] = calcWheelSlips(omega_rad, [vx_mps; vy_mps; dPsi_rad],...
  DeltaWheel_rad, tw_front_m, tw_rear_m, l_front_m, l_rear_m, tyreradius_front_m, tyreradius_rear_m);

[Fx_N, Fy_N] = TireModel(lambda_perc, alpha_rad, Fz_N, PacFrontLat, PacRearLat, PacFrontLong, PacRearLong);

%% calculate single track model 
% calculate equivalent single track accelerations
FxF_N = sum(Fx_N(1:2)); 
FxR_N = sum(Fx_N(3:4)); 
FyF_N = sum(Fy_N(1:2)); 
FyR_N = sum(Fy_N(3:4)); 
% calculate accelerations
ax_stm = (FxF_N*cos(DeltaWheel_rad) - FyF_N*sin(DeltaWheel_rad) + FxR_N - FxFriction + extForces_N(1))/m;
ay_stm = (FyF_N*cos(DeltaWheel_rad) + FxF_N*sin(DeltaWheel_rad) + FyR_N + extForces_N(2))/m; 
dvx_stm = ax_stm + dPsi_rad*vy_mps; 
dvy_stm = ay_stm - dPsi_rad*vx_mps; 
ddPsi_stm = ((FyF_N*cos(DeltaWheel_rad) + FxF_N*sin(DeltaWheel_rad))*l_front_m - FyR_N*l_rear_m + extTorques_Nm(3))/J; 
domega_stm = (FxPT_N - Fx_N).*wheelInertia_factors;

% calculate slip updates
dalpha_rad_stm = zeros(4,1);% 1/sample_Ti*(alpha_T_rad - alpha_rad); 
dlambda_perc_stm = zeros(4,1); %1/sample_Ti*(lambda_T_perc - lambda_perc); 


w_stm = 1; 
dvx = dvx_stm * w_stm;
dvy = dvy_stm * w_stm; 
ddPsi = ddPsi_stm * w_stm; 
domega_rad = domega_stm * w_stm; 
dalpha_rad = dalpha_rad_stm * w_stm; 
dlambda_perc = dlambda_perc_stm * w_stm; 
ax = ax_stm * w_stm; 
ay = ay_stm * w_stm; 

% write derivatives to output
ExactMeasurements = [vx_mps;vy_mps;dPsi_rad;ax;ay;omega_rad;]; 
DifferentialStates = [dvx;dvy;ddPsi;domega_rad;dlambda_perc;dalpha_rad;]; 
