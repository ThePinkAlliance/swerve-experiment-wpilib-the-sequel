// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class NavigateHolo extends CommandBase {
  private Pose2d currentPose = new Pose2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();

  private Trajectory trajectory;
  private int currentIndex = 0;

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final SwerveSubsystem swerveSubsystem;

  private boolean m_thetaEnabled = true;

  private final double MAX_VELOCITY = 3;
  private final double MAX_ACCELERATION = 3;

  private final double KP_X = 0.5;
  private final double KI_X = 0.0;
  private final double KD_X = 0.0;

  private final double KP_Y = 0.5;
  private final double KI_Y = 0.0;
  private final double KD_Y = 0.0;

  private final double KP_THETA = 0.5;
  private final double KI_THETA = 0.0;
  private final double KD_THETA = 0.0;

  /** Creates a new NavigateHolo. */
  public NavigateHolo(Trajectory trajectory, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.xController = new PIDController(KP_X, KI_X, KD_X);
    this.yController = new PIDController(KP_Y, KI_Y, KD_Y);
    this.thetaController = new ProfiledPIDController(KP_THETA, KI_THETA, KD_THETA,
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION), 0.02);

    this.swerveSubsystem = swerveSubsystem;
    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State desiredState = trajectory.getStates().get(currentIndex);
    Pose2d desiredPose = desiredState.poseMeters;
    Rotation2d desiredHeading = desiredPose.getRotation();
    ChassisSpeeds speeds;

    double desiredLinearVelocityMetersPerSecond = desiredState.velocityMetersPerSecond;

    // Calculate feedforward velocities (field-relative).
    double xFF = desiredLinearVelocityMetersPerSecond * desiredPose.getRotation().getCos();
    double yFF = desiredLinearVelocityMetersPerSecond * desiredPose.getRotation().getSin();
    double thetaFF = thetaController.calculate(
        currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    m_poseError = desiredPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), desiredPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), desiredPose.getY());

    SmartDashboard.putNumber("xFeedback", xFeedback);
    SmartDashboard.putNumber("yFeedback", yFeedback);

    if (!m_thetaEnabled) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));

    if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
      currentIndex++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentIndex > trajectory.getStates().size();
  }
}
