function marker_stamped_msg = update_marker_msg(marker_stamped_msg,pos,attitude,id)

marker_stamped_msg.pose.position.x = pos(1);
marker_stamped_msg.pose.position.y = pos(2);
marker_stamped_msg.pose.position.z = pos(3);
marker_stamped_msg.pose.orientation.w = attitude(1);
marker_stamped_msg.pose.orientation.x = attitude(2);
marker_stamped_msg.pose.orientation.y = attitude(3);
marker_stamped_msg.pose.orientation.z = attitude(4);
time = rostime('now');
marker_stamped_msg.header.stamp.sec = uint32(time.Sec);
marker_stamped_msg.header.stamp.nsec = uint32(time.Nsec);

if nargin == 3
    marker_stamped_msg.id = int32(0);
elseif nargin == 4
    marker_stamped_msg.id = int32(id);
else
    error("The input number is wrong \n")
end

end