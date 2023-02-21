function Position = GetTruePosition(PrePosition, velo, Tred, dt)
    u = CalcU(velo, Tred, dt);
    Position = CalcPosition(PrePosition, u);
end