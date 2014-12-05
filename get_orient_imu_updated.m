function imu = get_orient_imu(imu, varargin)

opt.method = 'simple';
opt.gyrooffset = [-16 -8];
opt.gyroband = [0.1 10];
opt.getoffset = true;

opt = parsevarargin(opt,varargin,2);

imu.rate = 1/mean(diff(imu.t));

%ideally we want a bandpass, but matlab doesn't do well with very
%low cutoffs, so we do a running mean type operation to get rid of
%the very low frequencies
if (opt.gyroband(1) > 0)
    gyrolo = get_low_baseline(imu.t, imu.gyro, opt.gyroband(1));
    
    gyros = imu.gyro - gyrolo;
else
    gyros = imu.gyro;
end
%and then a low pass filter to get rid of the high frequencies
[b,a] = butter(5,opt.gyroband(2)/(imu.rate/2), 'low');
gyros = filtfilt(b,a, gyros);
gyros = gyros - repmat(nanmean(gyros),[size(imu.gyro,1) 1]);

%and then integrate to get orientation
orient = cumtrapz(imu.t, gyros);


if opt.getoffset
    %filter the accelerometer signal to get an approximation of the gravity
    %vector
    [b,a] = butter(5,0.5/(imu.rate/2),'low');
    %% Update 1.01 - obtain angular acceleration by differentiating gyro
    % Note : gyro is a N X 3 vector
    angacclo            = diff(gyros)./repmat(diff(imu.t),1,3); % (N-1) X 3 vector
    % Approach 1 (may be wrong) - replicating the 1st element to make it N size
    angacclo            = [angacclo(1,:);angacclo];
    % Approach 2 curve fitting (correct)
    % Need to do this	

    % Convert the units first - deg to radians
    imu.gyrocross       = crossProductMatrix(gyros*pi/180);
    imu.angacclocross   = crossProductMatrix(angacclo*pi/180);
    imu.correctiondist  = [6.6, 11.4, -7]*1e-3;  % Correction distance (1,3) in meters
    for ii = 1:size(imu.acc,1)
        imu.newacc(ii,:)  = imu.acc(ii,:) + imu.correctiondist*...
            (imu.angacclocross(:,:,ii) + imu.gyrocross(:,:,ii)^2)';
    end
%     acclo = filtfilt(b,a,imu.acc);
    acclo = filtfilt(b,a,imu.newacc);
    
    %normalize to a unit vecotr
    mag = sqrt(sum(acclo.^2,2));
    acclo = acclo ./ repmat(mag,[1 3]);

    %try to get a good roll angle
    orienta(:,1) = (pi + unwrap(atan2(acclo(:,2),acclo(:,3)))) * 180/pi;
    if (any(orienta(:,1) > 345))
        orienta(:,1) = orienta(:,1) - 360;
    elseif (any(orienta(:,1) < -345))
        orienta(:,1) = orienta(:,1) + 360;
    end
    %try to get pitch angle
    orienta(:,2) = -(pi + unwrap(atan2(acclo(:,1),acclo(:,3)))) * 180/pi;
    if (any(abs(orienta(:,2)) > 345))
        orienta(:,2) = orienta(:,2) - sign(first(orienta(:,2),isfinite(orienta(:,2))))*360;
    end
    
    %now try to eyeball the gyro offset so that it matches the initial
    %accelerometer orientation estimates
    off = [0 0];
    done = false;
    plot(imu.t, orient(:,1:2) - repmat(off,[size(orient,1) 1]));
    addplot(imu.t, orienta, '--');

    legend('Gyro x','Gyro y','Acc x','Acc y');
    while ~done
        off(1) = input('Gyro x offset?');
        off(2) = input('Gyro y offset?');
        
        plot(imu.t, orient(:,1:2) - repmat(off,[size(orient,1) 1]));
        addplot(imu.t, orienta, '--');
        legend('Gyro x','Gyro y','Acc x','Acc y');
        
        done = inputyn('OK? ','default',true);
    end 
else
    off = opt.gyrooffset;
end

orient(:,1) = orient(:,1) - off(1);
orient(:,2) = orient(:,2) - off(2);

orient = orient * pi/180;

gvec = [sin(orient(:,2)) -sin(orient(:,1)) -cos(orient(:,2))];
gvecmag = sqrt(sum(gvec.^2,2));
gvec = gvec ./ repmat(gvecmag,[1 3]);

imu.orient = orient;
imu.acchi = imu.acc - gvec;

end

% Outputs the skew symmetric cross product matrix associated with cross
% product operation for a x b => [ax] b where [ax] is a skew symmetric
% matrix. Refer - http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
% 
% [theMatrix] = crossProductMatrix(theVector)
% Input = theVector - N X 3
% Output = theMatrix - 3 X 3 X N
function [theMatrix] = crossProductMatrix(theVector)
    N = size(theVector,1);
    theMatrix = zeros(3, 3, N);
    for ii = 1:N
        theMartix(:,:,ii) = [0, -theVector(ii,3), theVector(ii,2);
            theVector(ii,3), 0, -theVector(ii,1);
            -theVector(ii,2), theVector(ii,1), 0];
    end
end
