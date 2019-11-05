function robotCornerTrajectories = ComputeRobotCornerTrajectories(trajectory,robotLength,robotWidth,robotWheelRadius)
% We use basic trigonometry to compute the trajectory of the robots corners
% given the center and its heading
frontRightCornerTrajectory = trajectory(:,1:2) + robotWidth/2*[-sin(trajectory(:,3)),cos(trajectory(:,3))]+[cos(trajectory(:,3)),sin(trajectory(:,3))]*robotWheelRadius;
frontLeftCornerTrajectory = trajectory(:,1:2) - robotWidth/2*[-sin(trajectory(:,3)),cos(trajectory(:,3))]+[cos(trajectory(:,3)),sin(trajectory(:,3))]*robotWheelRadius;
rearRightCornerTrajectory = trajectory(:,1:2) + robotWidth/2*[-sin(trajectory(:,3)),cos(trajectory(:,3))]-[cos(trajectory(:,3)),sin(trajectory(:,3))]*(robotLength-robotWheelRadius);
rearLeftCornerTrajectory = trajectory(:,1:2) - robotWidth/2*[-sin(trajectory(:,3)),cos(trajectory(:,3))]-[cos(trajectory(:,3)),sin(trajectory(:,3))]*(robotLength-robotWheelRadius);
robotCornerTrajectories={frontRightCornerTrajectory,frontLeftCornerTrajectory,rearRightCornerTrajectory,rearLeftCornerTrajectory};