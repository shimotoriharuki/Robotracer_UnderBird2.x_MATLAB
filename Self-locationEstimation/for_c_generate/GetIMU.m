function theta = GetIMU(pre, u, Qt, dt) %�W���C������p�x���v�Z�����ꍇ
%     global Qt
%     global dt
    
    theta = pre + (u * dt + randn() * Qt); 
%     theta = pre + (u * dt + normrnd(0, sqrt(Qt))); 
end