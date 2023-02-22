function [EstPosition, EstPt] = GetSelfLocation(MeasuredPosition, ObsZt, TargetVelo, PrePosition, PrePt, ErrerParameter, Qt, Tred, dt)
    % --------------------Init------------------%
%     global ErrerParameter;
%     global Qt;
      
%     At = [0, 0, 0;
%           0, 0, 0;
%           0, 0, 0];
%       
%     Wt = [0, 0;
%           0, 0;
%           0, 0];
    
    Ht = [0, 0, 1];
    
    %---------------------- Calclation start----------------------------%
    % Calclate dS & dTh
    u = CalcU(TargetVelo, Tred, dt); 
    
    % The process noise covariance matrix
    Rt = CalcRt(ErrerParameter, u); 
    
    %----------------------- Forecast step----------------------%
    % Calclation At & Wt
    At = CalcAt(PrePosition, u);
    Wt = CalcWt(PrePosition, u);

    % Calclation estmation errors covariance matrix
    HatPt = At * PrePt * At' + Wt * Rt * Wt'; 

    
    %------------ Update step------------%
    % Get Robot's angle for gyro or geomagnetism
%     ObsZt = GetAngleForIMU(PreZt, TargetVelo(2), Qt, dt); 
    
    % Covariance of observation residuals
    St = Ht * HatPt * Ht' + Qt; 
    
    % Calclation Kalman constant
    Kt = St \ (HatPt * Ht');
    
    % Transpose matrix
    MeasuredPosition = MeasuredPosition';
    
    % Calclation estimation position
    EstPosition = MeasuredPosition + Kt * (ObsZt - Ht * MeasuredPosition); 
    
    % Updata estmation errors covariance matrix
    EstPt = (eye(size(EstPosition, 1)) - Kt * Ht) * HatPt; 
    
end