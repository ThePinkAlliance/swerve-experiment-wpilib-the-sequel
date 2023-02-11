// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.math.SphericalCoordinates;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.TurretSubsystem;

/**
 * The purpose of this command is to allow us to input a desired location in 3d
 * space and command to said position.
 */
public class ArmCoordinator extends CommandBase {
  SphericalCoordinates desiredLocationSpherical;

  ArmSubsystem armSubsystem;
  TurretSubsystem turretSubsystem;

  /** Creates a new ArmCoordinator. */
  public ArmCoordinator(Translation3d desiredLocation, ArmSubsystem armSubsystem, TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.desiredLocationSpherical = SphericalCoordinates.fromCartesian(desiredLocation);

    addRequirements(armSubsystem, turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SphericalCoordinates sphericalDifference = desiredLocationSpherical.subtract(getCurrentCoordinates());

    double pivotPower = armSubsystem.calculatePivotInput(sphericalDifference.getPhi());
    double turretPower = turretSubsystem.calculateTurretInput(sphericalDifference.getTheta());

    SmartDashboard.putNumber("Current Pivot", getCurrentCoordinates().getPhi());
    SmartDashboard.putNumber("Current Turret", getCurrentCoordinates().getTheta());
    SmartDashboard.putNumber("Current Radius", getCurrentCoordinates().getR());

    SmartDashboard.putNumber("Desired PivotPower", pivotPower);
    SmartDashboard.putNumber("Desired TurretPower", turretPower);
    SmartDashboard.putNumber("Desired Radius", sphericalDifference.getR());

    // turretSubsystem.commandTurret(turretPower);

    // armSubsystem.setExtenionDistance(sphericalDifference.getR());
    // armSubsystem.commandExtend(pivotPower);
  }

  private SphericalCoordinates getCurrentCoordinates() {
    return new SphericalCoordinates(armSubsystem.getExtensionDistance(), turretSubsystem.getTurretAngle(),
        armSubsystem.getPivotAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atExtensionSetpoint() && armSubsystem.atPivotSetpoint() && turretSubsystem.atTurretSetpoint();
  }
}
