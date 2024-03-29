package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.camera.CameraData;
import frc.robot.subsystems.camera.CameraData.TargetData;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;

public class AprilTagMoverCommand extends CommandBase {
    private final SwerveSubsystem driveSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private boolean findReflectiveTarget = false;

    public AprilTagMoverCommand(SwerveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(cameraSubsystem);
        cameraSubsystem.setPipeline(PipelineType.APRIL_TAG);
    }

    @Override
    public void execute() {
        /*
         * if (!findReflectiveTarget) {
         * driveCloserToTarget();
         * } else {
         * findReflectiveTarget();
         * }
         */
        driveCloserToTarget_NoRetro();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void driveCloserToTarget() {
        double rightDriveSpeed = 0;
        double leftDriveSpeed = 0;
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.APRIL_TAG) {
            if (camResult.hasTargets()) {
                double distance = camResult.getTargets().get(0).targetDistance;
                double angle = camResult.getTargets().get(0).targetXAngle;
                SwerveModuleState[] states = Constants.DriveConstants.kDriveKinematics
                        .toSwerveModuleStates(getHolonomicDriveValues(distance, angle));

                driveSubsystem.setModuleStates(states);
            }
        } else
            System.out.println("Waiting for pipeline to switch to APRIL TAG mode");
    }

    private void driveCloserToTarget_NoRetro() {
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.APRIL_TAG) {
            double translation = 0.0;
            double yAxisTranslation = 0.0;
            if (camResult.hasTargets()) {
                TargetData target = camResult.getTargets().get(0);
                if (target.targetDistance > 1)
                    translation = -.5;
                else if (target.targetDistance < .45)
                    translation = 0.4;
                else {
                    translation = 0.0;
                }
                double x = camResult.getTargets().get(0).targetXAngle;
                if (x > 2.0)
                    yAxisTranslation = 0.3;
                else if (x < -2.0)
                    yAxisTranslation = -0.3;
                else
                    yAxisTranslation = 0.0;
                SmartDashboard.putNumber("VX = ", translation);
                SmartDashboard.putNumber("XY", yAxisTranslation);
                SmartDashboard.putNumber("# Targets = ", camResult.getTargets().size());
                SmartDashboard.putNumber("Distance Meters = ", camResult.getTargets().get(0).targetDistance);
                SmartDashboard.putNumber("Angle Offset = ", x);
            }

            driveSubsystem.move(translation, yAxisTranslation, yAxisTranslation);
        }
    }

    private void findReflectiveTarget() {
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.REFLECTIVE) {
            if (camResult.hasTargets()) {
                if (camResult.getTargets().get(0).targetXAngle < -5)
                    driveSubsystem.move(-0.3, 0.3, 0);
                else if (camResult.getTargets().get(0).targetXAngle > 5)
                    driveSubsystem.move(0.3, -0.3, 0);
                else {
                    driveSubsystem.stopModules();
                    cameraSubsystem.setPipeline(PipelineType.APRIL_TAG);
                    findReflectiveTarget = false;
                }
            } else
                driveSubsystem.stopModules();
        } else
            System.out.println("Waiting for pipeline to switch to REFLECTIVE mode");
    }

    /**
     * Calculate the left and right drive power based on distance and angle.
     * 
     * @param distance distance
     * @param angle    angle
     * @return the power for the left and right drive
     */
    protected static Pair<Double, Double> getDiffDriveValues(double distance, double angle) {
        final double maxSpeed = 0.7;
        double speed = maxSpeed;
        if (distance > 4)
            speed = maxSpeed * 0.7;
        else if (distance > 2)
            speed = maxSpeed;
        else if (distance > 1.3)
            speed = maxSpeed * 0.8;
        else if (distance > 1)
            speed = maxSpeed * 0.5;
        else if (distance < 0.7)
            speed = -maxSpeed * 0.5;
        else
            speed = 0;

        double turnRatio = 0;
        if (Math.abs(angle) > 3) {
            turnRatio = 0.7;
        } else if (Math.abs(angle) > 7) {
            turnRatio = 0.9;
        }
        if (angle < 0)
            turnRatio *= -1;
        if (speed < 0)
            turnRatio *= -1;
        double rightDriveSpeed = speed + (speed * turnRatio);
        double leftDriveSpeed = speed - (speed * turnRatio);
        return new Pair<Double, Double>(leftDriveSpeed, rightDriveSpeed);
    }

    protected static ChassisSpeeds getHolonomicDriveValues(double distance, double angle) {
        double x = 0;
        double y = 0;
        double rot = 0;

        if (distance > 1) {
            x = -0.5;
        } else if (distance < 0.45) {
            x = 0.5;
        } else {
            x = 0;
        }

        if (Math.abs(angle) > 2) {
            rot = Math.copySign(0.5, angle);
        } else {
            rot = 0;
        }

        return new ChassisSpeeds(x, y, rot);
    }

}
