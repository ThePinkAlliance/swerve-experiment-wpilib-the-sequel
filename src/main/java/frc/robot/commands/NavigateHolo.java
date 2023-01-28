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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NavigateHolo extends CommandBase {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;
  private Pose2d currentPose = new Pose2d();

  private Trajectory trajectory;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private boolean m_firstRun = true;

  /** Creates a new NavigateHolo. */
  public NavigateHolo(Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_xController = new PIDController(0, 0, 0);
    this.m_yController = new PIDController(0, 0, 0);
    this.m_thetaController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0), 0.02);

    this.trajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  public ChassisSpeeds calculateSpeeds(double desiredLinearVelocityMetersPerSecond, State state,
      Rotation2d desiredHeading) {
    Pose2d trajectoryPose = state.poseMeters;

    // Calculate feedforward velocities (field-relative).
    double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    double thetaFF = m_thetaController.calculate(
        currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    m_poseError = trajectoryPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
