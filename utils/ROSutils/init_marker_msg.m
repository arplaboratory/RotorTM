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
marker_msg.header.stamp.sec = time.Sec;
marker_msg.header.stamp.nsec = time.Nsec;

switch nargin
    
    case 4 
        
        scale = [1,1,1];
        color = [1,0,0,0];    
        
    case 5

        color = [1,0,0,0];
    
    case 6 
        
    case 7
        
        marker_msg.mesh_resource = mesh_resource;
        
    otherwise
        error("There is not enough inputs. At least 4 arguments are required.")
        
end

marker_msg.header.frame_id = frame_id;
marker_msg.type = int32(type);
marker_msg.action = int32(action);
marker_msg.scale.x = scale(1);
marker_msg.scale.y = scale(2);
marker_msg.scale.z = scale(3);
marker_msg.color.a = single(color(1)); % Don't forget to set the alpha!
marker_msg.color.r = single(color(2));
marker_msg.color.g = single(color(3));
marker_msg.color.b = single(color(4));

end