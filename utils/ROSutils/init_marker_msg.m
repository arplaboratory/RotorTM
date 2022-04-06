function marker_msg = init_marker_msg(pubhandle,type, action, frame_id, scale, color, mesh_resource)
% init_marker_msg initialize the visualization_msgs marker with frame_id,
% type, action, scale, color, mesh_resource
%
% INPUTS:
% pubhandle - The handle of ROS publisher
% worldframe  - The name string of worldframe
% scale(optional)   - The scale of model. By default they are (1,1,1)
% color(optional)  - The color of model. By default they are (1,0,0,0)
% mesh_resource(optional) - user can customize the mesh of marker
%
% OUTPUTS:
% marker_msg  - The marker message

marker_msg = rosmessage(pubhandle);
time = rostime("now");
marker_msg.Header.Stamp.Sec = time.Sec;
marker_msg.Header.Stamp.Nsec = time.Nsec;

switch nargin
    
    case 4 
        
        scale = [1,1,1];
        color = [1,0,0,0];    
        
    case 5

        color = [1,0,0,0];
    
    case 6 
        
    case 7
        
        marker_msg.MeshResource = mesh_resource;
        
    otherwise
        error("There is not enough inputs. At least 4 arguments are required.")
        
end

marker_msg.Header.FrameId = frame_id;
marker_msg.Type = int32(type);
marker_msg.Action = int32(action);
marker_msg.Scale.X = scale(1);
marker_msg.Scale.Y = scale(2);
marker_msg.Scale.Z = scale(3);
marker_msg.Color.A = single(color(1)); % Don't forget to set the alpha!
marker_msg.Color.R = single(color(2));
marker_msg.Color.G = single(color(3));
marker_msg.Color.B = single(color(4));

end