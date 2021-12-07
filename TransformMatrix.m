function TM = TransformMatrix(a, alpha, d, theta)
% TransformMatrix calculates the transform matrix between links given the DH parameters

    TM = [cos(theta), -sin(theta), 0, a;
        sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
        sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
        0, 0, 0, 1];
end
