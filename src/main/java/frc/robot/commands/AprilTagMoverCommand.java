package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.camera.CameraData;
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
       /* if (!findReflectiveTarget) {
            driveCloserToTarget();
        } else {
            findReflectiveTarget();
        }
        */
        driveCloserToTarget_NoRetro();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void driveCloserToTarget() {
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.APRIL_TAG) {
            double translation = 0;
            if (camResult.hasTargets()) {
                if (camResult.getTargets().get(0).targetDistance > 1.1) translation = 0.15;
                else if (camResult.getTargets().get(0).targetDistance < 0.9) translation = -0.15;
                else {
                    cameraSubsystem.setPipeline(PipelineType.REFLECTIVE);
                    findReflectiveTarget = true;
                }
            }
            driveSubsystem.move(translation, 0, 0);
        } else System.out.println("Waiting for pipeline to switch to APRIL TAG mode");
    }
    private void driveCloserToTarget_NoRetro() {
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.APRIL_TAG) {
            double translation = 0.0;
            double yAxisTranslation = 0.0;
            if (camResult.hasTargets()) {
                if (camResult.getTargets().get(0).targetDistance > 1.1) translation = 0.3;
                else if (camResult.getTargets().get(0).targetDistance < 0.9) translation = -0.3;
                else {
                    translation = 0.0;
                }
                double x = camResult.getTargets().get(0).targetXAngle;
                if (x > 2.0) yAxisTranslation = -0.25;
                else if (x < -2.0) yAxisTranslation = 0.25;
                else yAxisTranslation = 0.0;
                SmartDashboard.putNumber("VX = ", translation);
                SmartDashboard.putNumber("XY", yAxisTranslation);
                SmartDashboard.putNumber("# Targets = ", camResult.getTargets().size());
                SmartDashboard.putNumber("Distance Meters = ", camResult.getTargets().get(0).targetDistance);
                SmartDashboard.putNumber("Angle Offset = ", x);
            }
            
            driveSubsystem.move(translation, yAxisTranslation, 0);
        } 
    }

    private void findReflectiveTarget() {
        CameraData camResult = cameraSubsystem.getTarget();
        if (camResult.pipelineType == PipelineType.REFLECTIVE) {
            if (camResult.hasTargets()) {
                /* TODO calculate vx, vy, radsPerSec */
                if (camResult.getTargets().get(0).targetXAngle < -5) driveSubsystem.move(0, 0.0, 0.0);
                else if (camResult.getTargets().get(0).targetXAngle > 5) driveSubsystem.move(0, 0.0, 0.0);
                else { 
                    driveSubsystem.stopModules();
                    cameraSubsystem.setPipeline(PipelineType.APRIL_TAG);
                    findReflectiveTarget = false;
                }
            } else driveSubsystem.stopModules();
        } else System.out.println("Waiting for pipeline to switch to REFLECTIVE mode");
    }

}
