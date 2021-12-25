function Transformation_Matrix = ForwardKinematicsEquation(T1, T2, T3, T4, T5, T6)

    % DH Parameters
    a = [0,172.5,0,0,0,0];
    d = [117.6,0,0,107.8,0,130.1];
    alpha = [90,0,90,-90,90,0];
    T = [T1, T2, T3, T4, T5, T6];

    % Initialize T0-1 matrix
    Transformation_Matrix = [
        cosd(T(1)), -cosd(alpha(1))*sind(T(1)), sind(alpha(1))*sind(T(1)), a(1)*cosd(T(1));
        sind(T(1)), cosd(alpha(1))*cosd(T(1)), -sind(alpha(1))*cosd(T(1)), a(1)*sind(T(1));
        0, sind(alpha(1)), cosd(alpha(1)), d(1);
        0, 0, 0, 1]; 

    % Multiply T0-1 matrix by next 5 matrices for matrix 6 to Base
    for i = 2:6
            Transformation_Matrix = Transformation_Matrix * [
                cosd(T(i)), -cosd(alpha(i))*sind(T(i)), sind(alpha(i))*sind(T(i)), a(i)*cosd(T(i));
                sind(T(i)), cosd(alpha(i))*cosd(T(i)), -sind(alpha(i))*cosd(T(i)), a(i)*sind(T(i));
                0, sind(alpha(i)), cosd(alpha(i)), d(i);
                0, 0, 0, 1]; 
    end
end