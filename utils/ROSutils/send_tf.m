function send_tf(pubhandle,tfstampedmsg,pos,attitude)

tfstampedmsg.Transform.Translation.X = pos(1);
tfstampedmsg.Transform.Translation.Y = pos(2);
tfstampedmsg.Transform.Translation.Z = pos(3);
tfstampedmsg.Transform.Rotation.W = attitude(1);
tfstampedmsg.Transform.Rotation.X = attitude(2);
tfstampedmsg.Transform.Rotation.Y = attitude(3);
tfstampedmsg.Transform.Rotation.Z = attitude(4);
tfstampedmsg.Header.Stamp = rostime('now');
sendTransform(pubhandle, tfstampedmsg);

end