// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Navigate extends CommandBase {
  SwerveSubsystem subsystem;
  SwerveModulePosition desiredPosition;

  double initalPosition = 0;
  double xSpeed;

  Pose2d initalPose;

  /** Creates a new Navigate. */
  public Navigate(SwerveSubsystem sub, SwerveModulePosition desiredPosition, double xSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);

    this.desiredPosition = desiredPosition;
    this.subsystem = sub;
    this.xSpeed = xSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<SwerveModulePosition> positions = this.subsystem.getPositions();
    initalPosition = Math.abs(positions.get(0).distanceMeters) + Math.abs(positions.get(1).distanceMeters) / 2;
    initalPose = subsystem.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem
        .setModuleStates(
            Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, 0, 0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem
        .setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double xDistance = subsystem.getPose().getX();
    double xTarget = initalPose.getX() + desiredPosition.distanceMeters;

    System.out.println("isFinished, target pose: " + xTarget);
    System.out.println("isFinished, current pose: " + xDistance);

    return xDistance >= xTarget;
  }
}
