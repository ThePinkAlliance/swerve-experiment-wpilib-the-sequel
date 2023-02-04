package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup(SwerveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        addCommands(new AprilTagMoverCommand(driveSubsystem, cameraSubsystem));
        // addCommands(new MoveByDistanceCommand(driveSubsystem, 10, 0.5));
        // addCommands(new WaitCommand(2));
        // addCommands(new AutoMoveCommand(driveSubsystem, 0.5, -0.5).withTimeout(3));
        // addCommands(new WaitCommand(2));
        // addCommands(new MoveByDistanceCommand(driveSubsystem, 5, 0.5));
        // addCommands(new WaitCommand(8));
        // addCommands(new WaitCommand(2));
        // addCommands(new AutoMoveCommand(driveSubsystem, -0.4, -0.4).withTimeout(1));
        // addCommands(new WaitCommand(2));
        // addCommands(new AutoMoveCommand(driveSubsystem, 0.4, 0.4).withTimeout(1));


    }
}
