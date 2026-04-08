function obj = rGetDistanceWhycon(obj)


    obj.Marker = obj.subMarker.LatestMessage;
    
    if isempty(obj.subMarker.LatestMessage) == 0

    % Convert quaternion to Euler angles in radians
    quat = [obj.Marker.Orientation.W
            obj.Marker.Orientation.X
            obj.Marker.Orientation.Y
            obj.Marker.Orientation.Z];

        
    eulXYZ = quat2eul(quat','XYZ');
    
    obj.DistanceWhycon.X(1)  = -obj.Marker.Position.Z;   
    obj.DistanceWhycon.X(2)  = obj.Marker.Position.X;   
    obj.DistanceWhycon.X(3)  = obj.Marker.Position.Y;   
    obj.DistanceWhycon.X(4)  = eulXYZ(1); % Roll   
    obj.DistanceWhycon.X(5)  = eulXYZ(2); % Pitch
    obj.DistanceWhycon.X(6)  = eulXYZ(3); % Yaw
    
    end

   

end
