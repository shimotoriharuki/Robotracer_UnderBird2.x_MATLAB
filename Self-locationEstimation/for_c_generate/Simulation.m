hold off
clear all

% Init
StartTime = 0;
ContinueTime = 60;  %[s]
 
dt = 0.01;   %[s]
Step = ceil((ContinueTime - StartTime) / dt);

% Input parameter Init

InputVelo = [1, 0.]; % [Transration (m/s), Rotation (rad/s)]
    
% Dispersion Init
ErrerParameter = [1, 0.1, 0.1, 0.1]; % Mobile robot related error parameters
Qt = 0.0001 ^2; % The measurement noise covariance matrix

% Position Init
TruePosition = [0, 0, 0]; % Robot's true position
PreTruePosition = [0, 0, 0];

EstPosition = [0, 0, 0]; % Robot's estimation position 
PreEstPosition = [0, 0, 0];

DR_OnlyPosition = [0, 0, 0]; % Robot's Dead-Reckoning position
PreDR_OnlyPosition = [0, 0, 0];

MeasuredPosition = [0, 0, 0];

EstPt = 0; % Estimation Pt
PrePt = 0;

ObsZt = 0; % Observed Zt
PreZt = 0;

% global Tred ;
Tred = 100e-3;  %[m]

for i = 1 : Step
    
    TruePosition = GetTruePosition(PreTruePosition, InputVelo, Tred, dt);
    
    u = CalcU(InputVelo, Tred, dt); % Calclate dS & dTh
    DR_OnlyPosition = GetDR_Position(PreDR_OnlyPosition, u, ErrerParameter); % Only Dead-Reckoning position
    
    MeasuredPosition = GetMeasuredPosition(PreEstPosition, u, ErrerParameter); % Measured position
    
    [EstPosition, EstPt, ObsZt] = GetSelfLocation(PreEstPosition, PrePt, PreZt, InputVelo, MeasuredPosition, ErrerParameter, ...
        Qt, Tred, dt); % Estimated position by EKF
    
    PreTruePosition = TruePosition;
    PreEstPosition = EstPosition;
    PreDR_OnlyPosition = DR_OnlyPosition;
    PrePt = EstPt;
    PreZt = ObsZt;
    
    % Animation
    if rem(i, 100)==0
        plot(TruePosition(:, 1), TruePosition(:, 2),'.blue'); hold on;
        plot(EstPosition(1), EstPosition(2),'.red'); hold on;
        plot(DR_OnlyPosition(1), DR_OnlyPosition(2),'.black'); hold on;
        axis equal;
        legend('True', 'Est');

        drawnow
    end
end