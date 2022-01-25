function marker_stamped_msg = update_marker_msg(marker_stamped_msg,pos,attitude,id)

marker_stamped_msg.Pose.Position.X = pos(1);
marker_stamped_msg.Pose.Position.Y = pos(2);
marker_stamped_msg.Pose.Position.Z = pos(3);
marker_stamped_msg.Pose.Orientation.W = attitude(1);
marker_stamped_msg.Pose.Orientation.X = attitude(2);
marker_stamped_msg.Pose.Orientation.Y = attitude(3);
marker_stamped_msg.Pose.Orientation.Z = attitude(4);
time = rostime('now');
marker_stamped_msg.Header.Stamp.Sec = uint32(time.Sec);
marker_stamped_msg.Header.Stamp.Nsec = uint32(time.Nsec);

if nargin == 3
    marker_stamped_msg.Id = int32(0);
elseif nargin == 4
    marker_stamped_msg.Id = int32(id);
else
    error("The input number is wrong \n")
end

end