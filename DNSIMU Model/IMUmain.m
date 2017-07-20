function [Accel_Bias_Instability, Gyro_Bias_Instability, WN_accel, WN_gyro, RW_accel, RW_gyro] = IMUmain(nacc,ngyro)

%%% Initialization and Sensor Characteristics

% Timespan 
t = 3600;
% Accelerometer Model Inputs
%Initialize DetModel Inputs
S_a = 0;
Bf_a = 0;

%Initialize StoModel Inputs
B_a = 0.01;
T_a = 500;
K_a = 0.01;
freq_a = 10;
bias_a = 0.001;
sig_meas_a = 0.0331;
rrw_a = 0.01;
sig_w_a = 0.01;

% Gyroscope Model Inputs
%Initialize DetModel Inputs
S_g = 0;
Bf_g = 0;

%Initialize StoModel Inputs
B_g = 0.0012;
T_g = 500;
K_g = 0;
freq_g = 100;
bias_g = 0;
sig_meas_g = 0.0331;
rrw_g = 0;
sig_w_g = 0;

%%% True angular velocity and specific force reading
nsamp_a = t*freq_a; %Number of Accelerometer samples
nsamp_g = t*freq_g; %Number of Gyroscope samples
sensordata_acc = linspace(1,t,nsamp_a)*0;
sensordata_gyro = linspace(1,t,nsamp_g)*0;

%%% Compute Deterministic Model Output
% Accelerometer Model Creation
Accel_matdet = zeros(nsamp_a,nacc);
for i = 1:nacc
    Accel_matdet(:,i) = SensorDetModel(sensordata_acc,S_a,Bf_a);
end

% Gyroscope Model Creation
Gyro_matdet = zeros(nsamp_g,ngyro);
for i = 1:ngyro
    Gyro_matdet(:,i) = SensorDetModel(sensordata_gyro,S_g,Bf_g);
end

%%% Compute Stochastic Model Output
% Accelerometer Model Creation
Accel_matstoc = zeros(nsamp_a,nacc);
for i = 1:nacc
    for j = 1:nsamp_a
        [Accel_matstoc(j,i),bias_a,rrw_a] = SensorStocModel(Accel_matdet(j,i),B_a,K_a,T_a,freq_a,bias_a,sig_meas_a,rrw_a,sig_w_a);
    end
end

% Gyroscope Model Creation
% Accelerometer Model Creation
Gyro_matstoc = zeros(nsamp_g,ngyro);
for i = 1:ngyro
    for j = 1:nsamp_g
        [Gyro_matstoc(j,i),bias_g,rrw_g] = SensorStocModel(Gyro_matdet(j,i),B_g,K_g,T_g,freq_g,bias_g,sig_meas_g,rrw_g,sig_w_g);
    end
end

%%% Kalman Filter result
% Accelerometer Averaged
Accel_matAvg = mean(Accel_matstoc,2);

% Accelerometer Averaged
Gyro_matAvg = mean(Gyro_matstoc,2);


%%% Compute Allan Deviation 
% Accelerometer 
% Averaged Allan Deviation
[Tallan_accel,sigma_accel] = allan(Accel_matAvg,freq_a,100);
% Individual Allan Deviations
[Tallan_accel2,sigma_accel2] = allan(Accel_matstoc(:,1:nacc),freq_a,100);
figure(1)
loglog(Tallan_accel,sigma_accel,Tallan_accel2,sigma_accel2)
grid on
title('Accelerometer Allan Variance')
xlabel('t [s]')
ylabel('Allan Standard Deviation')
legend('Averaged')

% Gyroscope
% Averaged Allan Deviation
[Tallan_gyro,sigma_gyro] = allan(Gyro_matAvg,freq_g,100);
% Individual Allan Deviations
[Tallan_gyro2,sigma_gyro2] = allan(Gyro_matstoc(:,1:ngyro),freq_g,100);
figure(2)
loglog(Tallan_gyro,sigma_gyro,Tallan_gyro2,sigma_gyro2)
grid on
title('Gyroscope Allan Variance')
xlabel('t [s]')
ylabel('Allan Standard Deviation')
legend('Averaged')

%%% Extract Performance Metrics
%Convert to Log Scale
Taccel_log = 10*log10(Tallan_accel);
Saccel_log = 10*log10(sigma_accel);
Tgyro_log = 10*log10(Tallan_gyro);
Sgyro_log = 10*log10(sigma_gyro);

%White Noise
WN_accel = 10^(interp1(Taccel_log,Saccel_log,10*log10(1))/10);
WN_gyro = 10^(interp1(Tgyro_log,Sgyro_log,10*log10(1))/10);

%Random Walk
RW_accel = 10^(interp1(Taccel_log,Saccel_log,10*log10(3))/10);
RW_gyro = 10^(interp1(Tgyro_log,Sgyro_log,10*log10(3))/10);

%Bias Instability
Accel_Bias_Instability = min(sigma_accel);
Gyro_Bias_Instability = min(sigma_gyro);
end