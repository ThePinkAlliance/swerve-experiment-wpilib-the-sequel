// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * The purpose of this command is to allow us to input a desired location in 3d
 * space and command to said position.
 */
public class ArmCoordinator extends CommandBase {
  Transform3d desiredLocation;

  ArmSubsystem armSubsystem;
  TurretSubsystem turretSubsystem;

  /** Creates a new ArmCoordinator. */
  public ArmCoordinator(Transform3d desiredLocation, ArmSubsystem armSubsystem, TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.desiredLocation = desiredLocation;

    addRequirements(armSubsystem, turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
