package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.camera.CameraData;
import frc.robot.subsystems.camera.CameraInterface;
import frc.robot.subsystems.camera.CameraInterfaceSim;
import frc.robot.subsystems.camera.LimeLightCamera;
import frc.robot.subsystems.camera.PhotonVisionCamera;
import frc.robot.subsystems.camera.CameraInterface.PipelineType;

public class CameraSubsystem extends SubsystemBase {
    /**
     * Camera type.
     */
    public enum CameraType {
        LIMELIGHT,
        PHOTON_VISION
    }

    private CameraInterface camera;

    public CameraSubsystem(CameraType type) {
        NetworkTableInstance.getDefault().flush();

        if (!Robot.isReal())
            camera = new CameraInterfaceSim();
        else if (type == CameraType.PHOTON_VISION)
            camera = new PhotonVisionCamera();
        else
            camera = new LimeLightCamera();
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
