clear all;
clc;

bag = rosbag('scan_drone_120_points.bag');
estimated_odom_topic = select(bag,'Topic','/ucl_0/vrpn_client/estimated_odometry');
traj_reference_topic = select(bag,'Topic','/ucl_0/autopilot/TrajectoryReference');

traj_messages = readMessages(traj_reference_topic,'DataFormat','struct');
odom_messages = readMessages(estimated_odom_topic,'DataFormat','struct');

%No. 2 in the above bag is to go to the starting position

for i = 1:122
   traj{i,1} = traj_messages{1,1}.Trajectory(i).Position.X;
   traj{i,2} = traj_messages{1,1}.Trajectory(i).Position.Y;
   traj{i,3} = traj_messages{1,1}.Trajectory(i).Position.Z;
   traj{i,4} = traj_messages{1,1}.Trajectory(i).TimeMilliseconds;
end

traj_mat = cellfun(@(tr) double(tr), traj);

for i = 1:1867
   odom{i,1} = odom_messages{i,1}.Pose.Pose.Position.X;
   odom{i,2} = odom_messages{i,1}.Pose.Pose.Position.Y;
   odom{i,3} = odom_messages{i,1}.Pose.Pose.Position.Z;
   odom{i,4} = 1000 * (estimated_odom_topic.MessageList{i,1} - traj_reference_topic.StartTime);
end    

odom_mat = cellfun(@(od) double(od), odom);

plot (traj_mat(:,4),traj_mat(:,1))
hold on
plot (traj_mat(:,4),traj_mat(:,2))
plot (traj_mat(:,4),traj_mat(:,3))
plot (odom_mat(:,4),odom_mat(:,1))
%hold on
plot (odom_mat(:,4),odom_mat(:,2))
plot (odom_mat(:,4),odom_mat(:,3))
hold off




