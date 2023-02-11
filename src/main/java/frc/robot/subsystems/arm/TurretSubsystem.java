// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  CANSparkMax rotateMotor;
  RelativeEncoder rotateEncoder;
  ProfiledPIDController controller;

  private double maxRotations = 0;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(int rotateMotorId, Constraints constraints) {
    this.rotateMotor = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
    this.rotateEncoder = this.rotateMotor.getEncoder();

    this.controller = new ProfiledPIDController(0, 0, 0, constraints);
  }

  public double calculateTurretInput(double desiredAngle) {
    double desiredRotations = desiredAngle * ((maxRotations / 2) / 180);

    if (desiredRotations > maxRotations) {
      desiredRotations = maxRotations;
    } else if (desiredRotations < 0) {
      desiredRotations = 0;
    }

    return this.controller.calculate(getTurretAngle(), desiredAngle);
  }

  public boolean atTurretSetpoint() {
    return controller.atSetpoint();
  }

  public void commandTurret(double input) {
    this.rotateMotor.set(input);
  }

  public double getTurretAngle() {
    return this.rotateEncoder.getPosition() * (180 / (maxRotations / 2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
