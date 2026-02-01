function [ followJointTrajectoryMsg ] = JointVec2FollowJointTrajectoryMsg(robot, q, t, qvel, qacc)
%  converts sequence of joint configuration q_1,...,q_m to  joint Trajectory message
% q : n x m , matrix of waypoints,  m joint space dimension, n #waypoints
% t : n x 1 vector of time stamps of motion in seconds
% qvel : n x m , matrix of joint velocities,  m joint space dimension, n #waypoints
% qacc : n x m , matrix of joint accelerations,  m joint space dimension, n #waypoints

if (nargin < 5)
    qacc=0*q;
end

if (nargin < 4)
    qvel=0*q;
end

qtol=-1;
qveltol=-1;
qacctol=-1;

jointConf=homeConfiguration(robot);
followJointTrajectoryMsg=ros2message('control_msgs/FollowJointTrajectoryGoal');
followJointTrajectoryMsg.trajectory=JointVec2JointTrajectoryMsg(robot, q, t, qvel, qacc);
for j=1:size(q,2)
    followJointTrajectoryMsg.goal_tolerance(j)=ros2message('control_msgs/JointTolerance');
    followJointTrajectoryMsg.path_tolerance(j)=ros2message('control_msgs/JointTolerance');
    % followJointTrajectoryMsg.GoalTimeTolerance(j)=rosmessage('std_msgs/Duration');
    followJointTrajectoryMsg.goal_time_tolerance.sec=int32(0);
    followJointTrajectoryMsg.goal_time_tolerance.nanosec=uint32(1e7);  % 10 ms
    followJointTrajectoryMsg.goal_tolerance(j).name=jointConf(j).JointName;
    followJointTrajectoryMsg.path_tolerance(j).name=jointConf(j).JointName;
    followJointTrajectoryMsg.goal_tolerance(j).position=qtol;
    followJointTrajectoryMsg.goal_tolerance(j).velocity=qveltol;
    followJointTrajectoryMsg.goal_tolerance(j).acceleration=qacctol;
    followJointTrajectoryMsg.path_tolerance(j).position=qtol;
    followJointTrajectoryMsg.path_tolerance(j).velocity=qveltol;
    followJointTrajectoryMsg.path_tolerance(j).acceleration=qacctol;

%     for i=1:size(q,1)
%        followJointTrajectoryMsg.GoalTolerance(j).Position(i)=qtol;
%        followJointTrajectoryMsg.GoalTolerance(j).Velocity(i)=qveltol;
%        followJointTrajectoryMsg.GoalTolerance(j).Acceleration(i)=qacctol;
%        followJointTrajectoryMsg.PathTolerance(j).Position(i)=qtol;
%        followJointTrajectoryMsg.PathTolerance(j).Velocity(i)=qveltol;
%        followJointTrajectoryMsg.PathTolerance(j).Acceleration(i)=qacctol;
%     end
end

end

