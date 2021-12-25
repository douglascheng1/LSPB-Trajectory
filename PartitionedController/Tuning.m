%Tuning Script
Kpconst = 30; % tune this value
Kvconst = 2*sqrt(Kpconst); 
Kp = Kpconst*eye(3);
Kv = Kvconst*eye(3);