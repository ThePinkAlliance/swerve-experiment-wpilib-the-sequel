package frc.robot.subsystems.camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class LimeLightCamera implements CameraInterface {

    private static String cameraName = "limelight";

    private NetworkTable getCamera() {
        return NetworkTableInstance.getDefault().getTable(cameraName);
    }

    @Override
    public boolean isConnected() {
        boolean isConnected = false;
        if(getCamera().containsKey("getpipe")) isConnected = true;
        return isConnected;
    }

    @Override
    public void close() {
        // Nothing to do.
    }

    /**
     * Gets the pipeline index from the PhotonVision camera.
     * @return the pipeline index
     */
    private int getPipelineIndex() {
        if (isConnected()) {
            return (int)getCamera().getEntry("getpipe").getDouble(0);
        } else throw new IllegalStateException("Limelight camera not connected");
    }

    @Override
    public PipelineType setPipeline(PipelineType type) {
        if (isConnected()) {
            int pipelineIndex = getPipelineIndex(type);
            getCamera().putValue("pipeline", NetworkTableValue.makeInteger(pipelineIndex));
        }
        
        return getPipelineType(getPipelineIndex());
    }

    @Override
    public CameraData getTarget() {
        CameraData camTargets = new CameraData();
        camTargets.isConnected = isConnected();
        if (camTargets.isConnected) {
            NetworkTable camera = getCamera();
            camTargets.pipelineType = getPipelineType(getPipelineIndex());
            camTargets.latencyMillis = camera.getEntry("tl").getDouble(0);
            boolean hasTargets = camera.getEntry("tv").getDouble(0) == 1;
            if (hasTargets) { 
                switch (camTargets.pipelineType) {
                    case APRIL_TAG:
                        getAprilTagTargets(camera, camTargets);
                    
                    case REFLECTIVE:
                        getReflectiveTarget(camera, camTargets);
                }
            }
        }
        toDashboard(camTargets);
        return camTargets;
    }

    /**
     * Get reflective target data.
     * @param camera the camera reference
     * @param camTargets
     * @return a list of 0 or more reflective targets from largest area to smallest.
     */
    private void getReflectiveTarget(NetworkTable camera, CameraData camTargets) {
        //TODO: How do we get multiple targets?
        camTargets.addReflectiveTarget(0, camera.getEntry("tx").getDouble(0), 
            camera.getEntry("ty").getDouble(0));
    }

    /**
     * Get april tag target data.
     * @param camera the camera reference
     * @param camTargets
     * @return a list of 0 or more april tag targets from closest to farthest.
     */
    private void getAprilTagTargets(NetworkTable camera, CameraData camTargets) {
        //TODO: How do we get multiple targets?
        double[] camPose = camera.getEntry("campose").getDoubleArray(new double[0]);
        camTargets.addAprilTagTarget(0, camera.getEntry("tx").getDouble(0), 
            camera.getEntry("ty").getDouble(0), Math.abs(camPose[2]));
    }
}
