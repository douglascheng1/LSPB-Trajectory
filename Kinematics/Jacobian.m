    % Jacobian computes the jacobian and determinant for the parameters
    % below
    
    % Sybolic parameters
    syms t1 dt1 t2 dt2 t3 dt3 t4 dt4 t5 dt5 t6 dt6 L1 L2 L3 L4
    t = [t1, t2, t3, t4, t5, t6];
    dt = [dt1, dt2, dt3, dt4, dt5, dt6];
    
    DHParameters = [0, 0, L1, t1;
    pi/2, 0, 0, t2;
    0, L2, 0, t3;
    pi/2, 0, L3, t4;
    -pi/2, 0, 0, t5;
    pi/2, 0, L4, t6];

    N = 6;
    
    R_06 = eye(3);
    T_06 = eye(4);
    w = cell(N+1,1);
    w{1} = [0;0;0];
    
    
    for k=1:N
        
        T_Matrix = [cos(DHParameters(k,4)), -cos(DHParameters(k,1))*sin(DHParameters(k,4)), sin(DHParameters(k,1))*sin(DHParameters(k,4)), (DHParameters(k,2))*cos(DHParameters(k,4));
            sin(DHParameters(k,4)), cos(DHParameters(k,1))*cos(DHParameters(k,4)), -sin(DHParameters(k,1))*cos(DHParameters(k,4)), (DHParameters(k,2))*sin(DHParameters(k,4));
            0, sin(DHParameters(k,1)), cos(DHParameters(k,1)), (DHParameters(k,3));
            0, 0, 0, 1];
        
        R_Matrix = T_Matrix(1:3,1:3);
        R_06 = R_06 * R_Matrix; % each 0 to N matrix
        T_06 = T_06 * T_Matrix;
        
        % Transpose for reference frames
        TransR_Matrix = R_Matrix.';
    
        thetaDot = dt(k);
        % Calculate omega (w) with respect to itself (NwN)
        w{k+1,1} = TransR_Matrix * w{k,1} + [0;0;thetaDot];
    end
    
    % YransformToZero holds 0T6
    Px = T_06(1,4);
    Py = T_06(2,4);
    Pz = T_06(3,4);
    
    % 0w6 is w{N+1,1}
    angularToZero = R_06*w{N+1,1};
    Wx = angularToZero(1);
    Wy = angularToZero(2);
    Wz = angularToZero(3);
    
    % Jacobian cell
    JacobianM = cell(N,N);
    
    for i = 1:N
        JacobianM{1,i} = simplify(diff(Px,t(i)));
        JacobianM{2,i} = simplify(diff(Py,t(i)));
        JacobianM{3,i} = simplify(diff(Pz,t(i)));
        JacobianM{4,i} = simplify(diff(Wx,dt(i)));
        JacobianM{5,i} = simplify(diff(Wy,dt(i)));
        JacobianM{6,i} = simplify(diff(Wz,dt(i)));
    end
 
    % Display Jacobian
    JM = cell2sym(JacobianM)
    
    % Calculate Determinant
    d = simplify(det(JM));
    determinant = d
