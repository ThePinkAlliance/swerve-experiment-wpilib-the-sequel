// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import org.photonvision.PhotonCamera;

public class AprilTagOdometryUpdater extends CommandBase {
  Consumer<Pose2d> poseConsumer;
  CameraSubsystem cameraSubsystem;
  PhotonCamera photonCamera;

  /** Creates a new AprilTagOdometryUpdater. */
  public AprilTagOdometryUpdater(Consumer<Pose2d> poseConsumer, CameraSubsystem cameraSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.cameraSubsystem = cameraSubsystem;
    this.poseConsumer = poseConsumer;

    this.photonCamera = new PhotonCamera(NetworkTableInstance.getDefault(), "test");

    addRequirements(cameraSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d transform = photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
    Pose2d currentPose2d = new Pose2d(transform.getX(), transform.getY(), Rotation2d.fromRadians(
        transform.getRotation().getAngle()));

    poseConsumer.accept(currentPose2d);
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
