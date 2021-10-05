function transformationProduct = wristToBase(dhParameters)
%wristToBase computes the homogenous transformation that relates he wrist
%frame to the base frane, given the DH parameters for an N DOF manipulator

    frameCount = size(dhParameters,1); %get # of frames by getting the # of rows of the DH table
    
    for i=1:frameCount %iterate through each frame
        % Extract DH parameters for each frame
        alpha = dhParameters(i,1);
        a = dhParameters(i,2);
        d = dhParameters(i,3);
        theta = dhParameters(i,4);
    
           
        transformMatrix = [ %calcuate Tranformation Matrix for frame
            cosd(theta), -sind(theta), 0, a;
            sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
            sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), cosd(alpha)*d;
            0, 0, 0, 1 ];
        
        % calculate each intermediate Transformation Matrix
        if i == 1
                transformationProduct = transformMatrix;
        else
                transformationProduct = transformationProduct * transformMatrix;
        end
    end
end
