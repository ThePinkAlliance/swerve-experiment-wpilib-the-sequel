// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwerveController extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;
  private double lastTime;

  /**
   * Constructs a new SwerveController that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path.
   * This is left to the user to do since it is not appropriate for paths with
   * nonstationary
   * endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to
   *                           provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller for the
   *                           robot's x position.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param desiredRotation    The angle that the drivetrain should be facing.
   *                           This is sampled at each
   *                           time step.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  public SwerveController(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "SwerveController"),
            requireNonNullParam(yController, "yController", "SwerveController"),
            requireNonNullParam(thetaController, "thetaController", "SwerveController")),
        desiredRotation,
        outputModuleStates,
        requirements);
  }

  /**
   * Constructs a new SwerveController that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path.
   * This is left to the user since it is not appropriate for paths with
   * nonstationary endstates.
   *
   * <p>
   * Note 2: The final rotation of the robot will be set to the rotation of the
   * final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each
   * timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier
   * for rotation
   * should be used.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to
   *                           provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param xController        The Trajectory Tracker PID controller for the
   *                           robot's x position.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  public SwerveController(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        outputModuleStates,
        requirements);
  }

  /**
   * Constructs a new SwerveController that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path-
   * this is left to the user, since it is not appropriate for paths with
   * nonstationary endstates.
   *
   * <p>
   * Note 2: The final rotation of the robot will be set to the rotation of the
   * final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each
   * timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier
   * for rotation
   * should be used.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to
   *                           provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param controller         The HolonomicDriveController for the drivetrain.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  public SwerveController(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        controller,
        () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        outputModuleStates,
        requirements);
  }

  /**
   * Constructs a new SwerveController that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw
   * module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path-
   * this is left to the user, since it is not appropriate for paths with
   * nonstationary endstates.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to
   *                           provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param controller         The HolonomicDriveController for the drivetrain.
   * @param desiredRotation    The angle that the drivetrain should be facing.
   *                           This is sampled at each
   *                           time step.
   * @param outputModuleStates The raw output module states from the position
   *                           controllers.
   * @param requirements       The subsystems to require.
   */
  public SwerveController(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Supplier<Rotation2d> desiredRotation,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveController");
    m_pose = requireNonNullParam(pose, "pose", "SwerveController");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveController");
    m_controller = requireNonNullParam(controller, "controller", "SwerveController");

    m_desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveController");

    m_outputModuleStates = requireNonNullParam(outputModuleStates, "outputModuleStates", "SwerveController");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    System.out.println("Command Freq: " + (curTime - lastTime));

    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
    lastTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
