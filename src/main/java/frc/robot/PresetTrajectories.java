package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class PresetTrajectories {
  public Trajectory pathToDock() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        3.0)
        .setKinematics(DriveConstants.kDriveKinematics);

    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1.8, 0),
            new Translation2d(2, 0)),
        new Pose2d(2.6, 0,
            Rotation2d.fromDegrees(0)),
        trajectoryConfig);
  }

  public Trajectory pathOverDock() {
    TrajectoryConfig trajectoryConfig2 = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond * 0.5,
        1)
        .setKinematics(DriveConstants.kDriveKinematics);

    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(2.5, 0),
            new Translation2d(3, 0)),
        new Pose2d(4.7, 0,
            Rotation2d.fromDegrees(0)),
        trajectoryConfig2);
  }
}
