package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AprilTagOdometryUpdater;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.Navigate;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Zero;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CameraSubsystem.CameraType;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ShuffleboardTab debugTab = Shuffleboard.getTab("debug");
        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
        private final CameraSubsystem cameraSubsystem = new CameraSubsystem(CameraType.LIMELIGHT);
        private SendableChooser<Trajectory> autoSendable = new SendableChooser<Trajectory>();

        private final Command autoMoveCommand = new AutoCommandGroup(swerveSubsystem, cameraSubsystem);

        public static PIDController xController = new PIDController(AutoConstants.kPXController, 0.5, 0);
        public static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        public static ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

        public RobotContainer() {
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
                cameraSubsystem.setDefaultCommand(new AprilTagOdometryUpdater(
                                (pose) -> swerveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp()),
                                cameraSubsystem));

                configureButtonBindings();

                SmartDashboard.putData("Auto Chooser", autoSendable);

                debugTab.add(xController);
                debugTab.add(yController);
                debugTab.add(thetaController);

                try {
                        Trajectory idk = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/idk.wpilib.json"));
                        Trajectory e1 = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/E1.wpilib.json"));
                        Trajectory e2 = TrajectoryUtil.fromPathweaverJson(
                                        Filesystem.getDeployDirectory().toPath().resolve("output/E2.wpilib.json"));

                        autoSendable.addOption("e1", e1);
                        autoSendable.addOption("e2", e2);
                        autoSendable.setDefaultOption("idk", idk);
                } catch (Exception err) {
                        err.printStackTrace();
                }

                SmartDashboard.putNumber("distance", 2);
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoytick, 4).onTrue(
                                new Navigate(swerveSubsystem, new SwerveModulePosition(getDistance(), new Rotation2d()),
                                                1));
                new JoystickButton(driverJoytick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                new JoystickButton(driverJoytick, 2).onTrue(new Zero(swerveSubsystem));
        }

        private double getDistance() {
                return SmartDashboard.getNumber("distance", 2);
        }

        public Command getAutonomousCommand() {
                // return new AprilTagMoverCommand(swerveSubsystem, cameraSubsystem);
                Trajectory trajectory = autoSendable.getSelected();

                // Reset the swerve subsystem pose to the inital pose of the trajectory.
                swerveSubsystem.resetOdometry(trajectory.getInitialPose());

                // 4. Construct command to follow trajectory
                Command swerveControllerCommand = swerveSubsystem.buildSwerveCommand(trajectory);

                // 5. Add some init and wrap-up, and return everything
                return new SequentialCommandGroup(
                                new InstantCommand(() -> SmartDashboard.putString("InitalPose",
                                                swerveSubsystem.getPose()
                                                                .toString()),
                                                swerveSubsystem),
                                swerveControllerCommand.alongWith(),
                                new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
        }
}
