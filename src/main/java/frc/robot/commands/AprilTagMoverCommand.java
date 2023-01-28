package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to keep the robot within 1m of AprilTag.
 */
public class AprilTagMoverCommand extends CommandBase {
    private final SwerveSubsystem driveSubsystem;
    double xSpeed = 0;

    public AprilTagMoverCommand(SwerveSubsystem driveSubsystem, double xSpeed) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        NetworkTable cam = NetworkTableInstance.getDefault().getTable("limelight");
        // Do we have a camera connected?
        if(cam.containsKey("getpipe")) {
            SmartDashboard.putBoolean("Cam connected", true);
            boolean hasTarget = false;
            if (cam.getEntry("tv").getDouble(0) == 1) hasTarget = true;
            SmartDashboard.putBoolean("Cam has target", hasTarget);
            if (hasTarget) {
                SmartDashboard.putNumber("Cam latency", cam.getEntry("tl").getDouble(0));
                SmartDashboard.putNumber("Cam Tag ID", cam.getEntry("tid").getDouble(0));
                // Make sure we get some real data.
                double[] camPose = cam.getEntry("campose").getDoubleArray(new double[0]);
                if (camPose.length >= 3) {
                    SmartDashboard.putNumber("Cam X", camPose[0]);
                    SmartDashboard.putNumber("Cam Y", camPose[1]);
                    SmartDashboard.putNumber("Cam Z", camPose[2]);

                    // Here is the magic.
                    double distanceToTarget = Math.abs(camPose[2]);
                    if (distanceToTarget > 1.1) {
                        //driveSubsystem.move(0.3, 0.3);
                        driveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, 0, 0)));
                    }
                    else if (distanceToTarget < 0.9) {
                        //driveSubsystem.move(-0.3, -0.3);
                        driveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, 0, 0)));
                    }
                    else {
                        //driveSubsystem.move(0, 0);
                        driveSubsystem.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, 0, 0)));
                    }
                }
            } else {
                driveSubsystem.stopModules();
            }
        } else {
            SmartDashboard.putBoolean("Cam connected", false);
        }

    }
}
