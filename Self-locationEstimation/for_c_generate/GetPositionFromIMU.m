function position = GetPositionFromIMU(PrePosition, TargetVelo, Angle, dt)

    delta_distance = TargetVelo(1) * dt;
%     delta_theta = TargetVelo(2) 

    position(1) = PrePosition(1) + delta_distance * cos(Angle);
    position(2) = PrePosition(2) + delta_distance * sin(Angle);

end