// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX pivotMotor;
  CANSparkMax extendMotor;
  RelativeEncoder extendEncoder;
  CANCoder canCoder;
  ProfiledPIDController pivotController;

  private double maxRotations = 0;
  private double maxDistanceMeters = 0;
  private double maxPivotAngle = 270;
  private double minPivotAngle = 27;
  private double pivotOffset;
  private double desiredRotations = 0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int pivotMotorId, int extendMotorId, int canCoderId, double pivotOffset,
      Constraints constraints) {
    this.pivotMotor = new TalonFX(pivotMotorId);
    this.extendMotor = new CANSparkMax(extendMotorId, MotorType.kBrushless);
    this.canCoder = new CANCoder(canCoderId);
    this.pivotController = new ProfiledPIDController(0, 0, 0, constraints);

    this.extendEncoder = extendMotor.getEncoder();
    this.pivotOffset = pivotOffset;
  }

  public double calculatePivotInput(double angle) {
    if (angle > maxPivotAngle) {
      angle = maxPivotAngle;
    } else if (angle < minPivotAngle) {
      angle = minPivotAngle;
    }

    return pivotController.calculate(getPivotAngle(), angle);
  }

  public void setExtenionDistance(double distance) {
    double desiredRotations = distance * (maxRotations / maxDistanceMeters);

    // This will clip the commandable rotations between 0 and maxRotations.
    if (desiredRotations > maxRotations) {
      desiredRotations = maxRotations;
    } else if (desiredRotations < 0) {
      desiredRotations = 0;
    }

    this.desiredRotations = desiredRotations;

    extendMotor.getPIDController().setReference(desiredRotations, ControlType.kPosition);
  }

  public boolean atExtensionSetpoint() {
    double tolerence = 0.3;
    double extensionDistance = getExtensionDistance();

    if ((desiredRotations - tolerence) < extensionDistance && extensionDistance < (desiredRotations + tolerence))
      return true;
    else
      return false;
  }

  public boolean atPivotSetpoint() {
    return pivotController.atSetpoint();
  }

  public void commandExtend(double input) {
    this.extendMotor.set(input);
  }

  public void commandPivot(double input) {
    this.pivotMotor.set(ControlMode.PercentOutput, input);
  }

  /**
   * Returns the extended distance in meters.
   */
  public double getExtensionDistance() {
    return extendEncoder.getPosition() * (maxDistanceMeters / maxRotations);
  }

  public double getPivotAngle() {
    return canCoder.getPosition() + pivotOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
