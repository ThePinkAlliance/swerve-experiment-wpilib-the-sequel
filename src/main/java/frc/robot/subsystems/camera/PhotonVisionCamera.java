package frc.robot.subsystems.camera;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Fetch the best target from a PhotonVision camera.
 * Assumption is that the April Tag pipeline is pipeline 0 and
 * reflective if 1.
 */
public class PhotonVisionCamera implements CameraInterface {
    private PhotonCamera camera;
    private static String cameraName = "main";

    public PhotonVisionCamera() {
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    
    @Override
    public void close() {
        if (isConnected()) camera.close();
    }

    /**
     * Gets the pipeline index from the PhotonVision camera.
     * @return the pipeline index
     */
    private int getPipelineIndex() {
        return camera.getPipelineIndex();
    }

    @Override
    public PipelineType setPipeline(PipelineType type) {
        camera.setPipelineIndex(getPipelineIndex(type));
        return getPipelineType(getPipelineIndex());
    }

    @Override
    public CameraData getTarget() {
        CameraData camTargets = new CameraData();
        camTargets.isConnected = camera.isConnected();
        if (camTargets.isConnected) {
            camTargets.pipelineType = getPipelineType(getPipelineIndex());
            PhotonPipelineResult camResults = camera.getLatestResult(); 
            camTargets.latencyMillis = camResults.getLatencyMillis();
            if (camResults.hasTargets()) { 
                switch (camTargets.pipelineType) {
                    case APRIL_TAG:
                        getAprilTagTargets(camResults, camTargets);
                    
                    case REFLECTIVE:
                        getReflectiveTarget(camResults, camTargets);
                }
            }
        }
        toDashboard(camTargets);
        return camTargets;
    }

    /**
     * Get reflective target data.
     * @param results the reflective target data
     * @param camTargets
     * @return a list of 0 or more reflective targets from largest area to smallest.
     */
    private void getReflectiveTarget(PhotonPipelineResult results, CameraData camTargets) {
        int targetIndex = 0;
            
        // Sort by largest
        List<PhotonTrackedTarget> photonTargets = results.getTargets();
        Collections.sort(photonTargets, new Comparator<PhotonTrackedTarget>() {
            @Override
            public int compare(PhotonTrackedTarget arg0, PhotonTrackedTarget arg1) {
                double diff = arg0.getArea() - arg1.getArea();
                if (diff > 0) return 1;
                else if (diff < 0) return -1;
                else return 0;
            }
        }); 

        // Now add the targets
        for (PhotonTrackedTarget target : photonTargets) {
            camTargets.addReflectiveTarget(targetIndex++, target.getYaw(), target.getPitch());
        } 
    }

    /**
     * Get april tag target data.
     * @param results the april tag target data
     * @param camTargets
     * @return a list of 0 or more april tag targets from closest to farthest.
     */
    private void getAprilTagTargets(PhotonPipelineResult results, CameraData camTargets) {
        // Sort by closest
        List<PhotonTrackedTarget> photonTargets = results.getTargets();
        Collections.sort(photonTargets, new Comparator<PhotonTrackedTarget>() {
            @Override
            public int compare(PhotonTrackedTarget arg0, PhotonTrackedTarget arg1) {
                double diff = arg0.getBestCameraToTarget().getX() - arg1.getBestCameraToTarget().getX();
                if (diff > 0) return 1;
                else if (diff < 0) return -1;
                else return 0;
            }
        }); 

        // Now add the targets
        for (PhotonTrackedTarget target : photonTargets) {
            camTargets.addAprilTagTarget(target.getFiducialId(), target.getYaw(), target.getPitch(), target.getBestCameraToTarget().getX());
        }
    }

    @Override
    protected void finalize() throws Throwable {
        if (camera.isConnected()) camera.close();
    }
}
