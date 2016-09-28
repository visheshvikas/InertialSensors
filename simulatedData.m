% This function generates simulated IMU data of a virtual fish

function [simimu] = simulatedData(timelength,varargin)
Aamp = 0;
if(not(isempty(varargin)))
    for ii=1:2:length(varargin)
        if(strcmp(varargin(ii),'Aamp'))
            Aamp = cell2mat(varargin(ii+1));
        end
    end
end
% keyboard
calibmillisecs = 1000;
% timelingth = 4;
samplefreq	= 1e-3;     % 1 KHz
time        = 0:samplefreq:timelength; % in seconds
% Accelerometer (datesheet) quantities
accel.noisestd  = (3e-4*sqrt(0.5/samplefreq));          % unit = g (9.81 m/s2)
% Gyroscope (datasheet) quantities
gyro.noisestd   = deg2rad(0.01*sqrt(0.5/samplefreq)); % unit = radians
gyro.biasstd    = gyro.noisestd;
% Simulation characteristics
freq        = 0.5;


simimu.Qgyro    = (gyro.noisestd)^2*eye(3);
simimu.Qacc     = (accel.noisestd)^2*eye(3);
simimu.Qbias    = (gyro.biasstd)^2*eye(3);
simimu.Qacc     = 0.1*eye(3);

roll.amplitude  = deg2rad(10);
roll.phase      = deg2rad(25);
roll.freq       = freq;
roll.angle      = roll.amplitude*sin(2*pi*roll.freq*time + roll.phase);
roll.rate       = 2*pi*roll.freq*roll.amplitude*cos(2*pi*roll.freq*time + roll.phase);
roll.rateofrate = -(2*pi*roll.freq)^2*roll.angle;

pitch.amplitude = 0;
pitch.angle     = zeros(size(time));
pitch.rate      = zeros(size(time));
pitch.rateofrate = zeros(size(time));

yaw.amplitude   = deg2rad(20);
yaw.phase       = deg2rad(40);
yaw.freq        = freq;
yaw.angle       = yaw.amplitude*sin(2*pi*yaw.freq*time + yaw.phase);
yaw.rate        = 2*pi*yaw.freq*yaw.amplitude*cos(2*pi*yaw.freq*time + yaw.phase);
yaw.rateofrate  = -(2*pi*yaw.freq)^2*yaw.angle;


% 
roll.angle      = [zeros(1,calibmillisecs), roll.angle];
pitch.angle     = [zeros(1,calibmillisecs), pitch.angle];
yaw.angle       = [zeros(1,calibmillisecs), yaw.angle];
roll.rate      = [zeros(1,calibmillisecs), roll.rate];
pitch.rate     = [zeros(1,calibmillisecs), pitch.rate];
yaw.rate       = [zeros(1,calibmillisecs), yaw.rate];
roll.rateofrate      = [zeros(1,calibmillisecs), roll.rateofrate];
pitch.rateofrate     = [zeros(1,calibmillisecs), pitch.rateofrate];
yaw.rateofrate       = [zeros(1,calibmillisecs), yaw.rateofrate];

% Converting Roll-Pitch-Yaw into Euler angles
% RPY = ZYX Euler Angles = R(x,phi)*R(y,theta)*R(z,psi)
% psi = Yaw, theta = Pitch, phi = Roll
psi     = yaw;
theta   = pitch;
phi     = roll;
 [Omega_Sensor, Alpha_Sensor] = ...
    getRotationRatesEulerZYX(psi, theta, phi);
simimu.realeulerrad = [phi.angle;theta.angle;psi.angle];
%simimu.quaternion = angle2quat(psi.angle',theta.angle',phi.angle','ZYX');

% Rotation of the rigid body
Ax          = Aamp*(accel.noisestd);
% Ax          = 0;
rOP         = [0,0,0]'; % meters
Accel_CoM_Inertial  = [Ax*sin(4*pi*freq*time);zeros(2,length(time))];
Accel_CoM_Inertial  = [zeros(3,calibmillisecs),Accel_CoM_Inertial];
simimu.dynaccGlobal = Accel_CoM_Inertial;
Accel_CoM_Inertial  = Accel_CoM_Inertial + repmat([0 0 -1]',1, length(Accel_CoM_Inertial));
Accel_CoM_Sensor    = inertial2sensor(Accel_CoM_Inertial, roll.angle, pitch.angle, yaw.angle);
Accel_P     = getAcceleration(Accel_CoM_Sensor, rOP, Omega_Sensor, Alpha_Sensor);

% Adding noise
% gyro.sigma = (gyro.noisevar)*eye(3,3); R = (chol(gyro.sigma));
simimu.gyro     = Omega_Sensor + (gyro.noisestd)*randn(size(Omega_Sensor));
% Adding bias drift
Bias            = getBias(length(Omega_Sensor), gyro.biasstd);
simimu.gyro     = simimu.gyro+Bias;
% accel.sigma = (accel.noisevar)*eye(3,3); R = chol(accel.sigma);
simimu.acc  = Accel_P + accel.noisestd*randn(size(Accel_P));


time = 0:samplefreq:(timelength+calibmillisecs*samplefreq);



simimu.gyro     = simimu.gyro';
simimu.acc      = simimu.acc';
simimu.t        = time';
simimu.sampfreq = samplefreq;
simimu.gyronoisestd = gyro.noisestd;
simimu.gyrobiasdriftstd = gyro.biasstd;
simimu.accnoisestd = accel.noisestd;
figure('Name','Simulated Sensor Data')
% Plot Gyroscope
subplot(2,2,1)
plot(time, rad2deg(Omega_Sensor));
title('Ideal Gyroscope readings');
legend('Sensor X', 'Sensor Y', 'Sensor Z')
xlabel('time (seconds)'); ylabel('degrees/sec');
gylim1 = get(gca,'ylim');
subplot(2,2,2)
plot(time, rad2deg(simimu.gyro));
title('Simulated Gyroscope with drift');
legend('Sensor X', 'Sensor Y', 'Sensor Z')
xlabel('time (seconds)'); ylabel('degrees/sec');
gylim2 = get(gca,'ylim');
gylim = [min([gylim1(1),gylim2(1)]), max(gylim1(2), gylim2(2))];
set(gca,'ylim',gylim);
subplot(2,2,1)
set(gca,'ylim',gylim);

% Plot Accelerometer
subplot(2,2,3)
plot(time, Accel_P);
title('Ideal Accelerometer readings');
legend('Sensor X', 'Sensor Y', 'Sensor Z')
xlabel('time (seconds)'); ylabel('g m/s^2');
aylim1 = get(gca,'ylim');
subplot(2,2,4)
plot(time, simimu.acc);
title('Simulated Acceleroemter');
legend('Sensor X', 'Sensor Y', 'Sensor Z')
xlabel('time (seconds)'); ylabel('g m/s^2');
aylim2 = get(gca,'ylim');
aylim =[min([aylim1(1),aylim2(1)]), max(aylim1(2), aylim2(2))];
set(gca,'ylim',aylim);
subplot(2,2,3)
set(gca,'ylim',aylim);

end

function [Omega_Sensor, Alpha_Sensor] = ...
    getRotationRatesEulerZYX(psi, theta, phi)
% 
% phi = roll; theta = pitch; psi = yaw;
    N = length(psi.angle);
    Omega_Sensor = zeros(3,N);
    Alpha_Sensor = zeros(3,N);
    for ii=1:N
        Theta_d     = [phi.rate(ii); theta.rate(ii); psi.rate(ii)];
        Theta_dd    = [phi.rateofrate(ii); theta.rateofrate(ii); psi.rateofrate(ii)];
        sTh         = sin(theta.angle(ii)); 
        cTh         = cos(theta.angle(ii));
        sPh         = sin(phi.angle(ii)); 
        cPh         = cos(phi.angle(ii));
        W           = [1, 0, -sTh;
                       0, cPh, cTh*sPh;
                       0, -sPh, cTh*cPh];
        W_phi       = [0,0,0;
                       0,-sPh, cTh*cPh;
                       0,-cPh, -cTh*sPh];
        W_theta     = [0,0,-cTh;
                       0,0,-sTh*sPh;
                       0,0,-sTh*cPh];
        Omega_Sensor(:,ii)     = W*Theta_d;
        Alpha_Sensor(:,ii)     = [W_phi*Theta_d, W_theta*Theta_d, zeros(3,1)]*Theta_d + W*Theta_dd;
    end
end

% Two points on a rigid body
function [Accel_P] = getAcceleration(Accel_O, rOP, Omega_Sensor, Alpha_Sensor)
    N = length(Accel_O);
    Accel_P = zeros(3,N);

    for ii=1:N
        Accel_P(:,ii) = Accel_O(:,ii) + ...
            (crossMat(Alpha_Sensor(:,ii)) + ...
            crossMat(Omega_Sensor(:,ii))^2)*rOP;
    end
end

% Cross Product matrix
function [Mat] = crossMat(vector)
    Mat = [0, -vector(3), vector(2);
        vector(3), 0, -vector(1);
        -vector(2), vector(1), 0];
end

% Random walk
function [Bias] = getBias(length, std)
    Bias = zeros(3,length);
    Bias(:,1) = std*randn(3,1);
    for ii=2:length
        Bias(:,ii) = Bias(:,ii-1) + std*randn(3,1);
    end
end
% 
function [V_Sensor] = inertial2sensor(V_Inertial, roll, pitch, yaw)
NN = length(V_Inertial);
V_Sensor = zeros(3,NN);
    for ii = 1:NN
        V_Sensor(:,ii) = getQT([roll(ii), pitch(ii), yaw(ii)])*V_Inertial(:,ii);
    end
end
% Rotation Matrix
function [QT] = getQT(Theta)
    phi = Theta(1); theta = Theta(2); psi = Theta(3);
    Rz_yaw      = [cos(psi), sin(psi), 0;
                   -sin(psi), cos(psi), 0;
                   0 ,0, 1];
    Ry_pitch    = [cos(theta), 0 ,-sin(theta);
                   0, 1,0;
                   sin(theta), 0, cos(theta)];
    Rx_roll     = [1, 0, 0;
                    0, cos(phi), sin(phi);
                    0, -sin(phi), cos(phi)];
    QT              = Rx_roll*Ry_pitch*Rz_yaw;
end
