function marker_stamped_msg = update_line_list_msg(marker_stamped_msg,point_list,id)

% Assign Time Stamp
time = rostime('now');
marker_stamped_msg.header.stamp.sec = uint32(time.Sec);
marker_stamped_msg.header.stamp.nsec = uint32(time.Nsec);

[~,col] = size(point_list);
for i = 1:col
    marker_stamped_msg.points(i).x = point_list(1,i);
    marker_stamped_msg.points(i).y = point_list(2,i);
    marker_stamped_msg.points(i).z = point_list(3,i);
end

if nargin == 2
    marker_stamped_msg.id = int32(0);
elseif nargin == 3
    marker_stamped_msg.id = int32(id);
else
    error("The input number is wrong \n")
end


end