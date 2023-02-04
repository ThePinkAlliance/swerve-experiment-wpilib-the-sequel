package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.camera.CameraData;
import frc.robot.subsystems.camera.CameraInterface;
import frc.robot.subsystems.camera.CameraInterfaceSim;
import frc.robot.subsystems.camera.LimeLightCamera;
import frc.robot.subsystems.camera.PhotonVisionCamera;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;

public class CameraSubsystem extends SubsystemBase {
    // private CameraInterface camera = new PhotonVisionCamera();
    private CameraInterface camera = new LimeLightCamera();

    public CameraSubsystem() {
        if (!Robot.isReal())
            camera = new CameraInterfaceSim();
    }

    /**
     * Get closest target data.
     * 
     * @return closest target data
     */
    public CameraData getTarget() {
        return camera.getTarget();
    }

    /**
     * Set the pipeline type.
     * 
     * @param type the pipeline type
     */
    public void setPipeline(PipelineType type) {
        camera.setPipeline(type);
    }

    /**
     * Close the connection to the camera.
     */
    public void close() {
        camera.close();
    }
}
