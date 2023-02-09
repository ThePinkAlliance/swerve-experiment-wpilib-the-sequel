import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.math.SphericalCoordinates;
import org.junit.jupiter.api.Test;

public class SphericalCoordinatesTest {
  @Test
  public void sphericalToCartesian() {
    SphericalCoordinates coordinates = new SphericalCoordinates(7.34, -2.67, 2.83);
    Translation3d translation3d = coordinates.toCartesian();

    assertEquals(-2.004635177, translation3d.getX(), 0.05, "X Coordinate Matches");
    assertEquals(-1.022306058, translation3d.getY(), 0.05, "Y Coordinate Matches");
    assertEquals(-6.98655338, translation3d.getZ(), 0.05, "Z Coordinate Matches");
  }

  @Test
  public void cartesianToSpherical() {
    Translation3d translation3d = new Translation3d(-2, -1, -7);
    SphericalCoordinates sphericalCoordinates = SphericalCoordinates.fromCartesian(translation3d);

    assertEquals(7.3484, sphericalCoordinates.getR(), 0.05, "R Coordinate Matches");
    assertEquals(-2.6779, sphericalCoordinates.getTheta(), 0.05, "Theta Coordinate Matches");
    assertEquals(2.8323, sphericalCoordinates.getPhi(), 0.05, "Phi Coordinate Matches");
  }
}
