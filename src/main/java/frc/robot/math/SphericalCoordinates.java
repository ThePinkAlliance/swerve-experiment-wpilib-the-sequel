package frc.robot.math;

import edu.wpi.first.math.geometry.Translation3d;

public class SphericalCoordinates {
  private final double phi;
  private final double theta;
  private final double projection;

  public SphericalCoordinates(double phi, double theta, double projection) {
    this.phi = phi;
    this.projection = projection;
    this.theta = theta;
  }

  public double getTheta() {
    return theta;
  }

  public double getPhi() {
    return phi;
  }

  public double getProjection() {
    return projection;
  }

  /**
   * Convert from cartesian space to spherical space.
   * 
   * @param transform3d
   */
  public static SphericalCoordinates fromCartesian(Translation3d transform3d) {
    double x2 = transform3d.getX() * transform3d.getX();
    double y2 = transform3d.getY() * transform3d.getY();
    double z2 = transform3d.getZ() * transform3d.getZ();

    double theta = Math.tan(transform3d.getY() / transform3d.getX());
    double phi = Math.acos(transform3d.getZ() / (Math.sqrt(x2 + y2 + z2)));
    double projection = Math.sqrt(x2) + Math.sqrt(y2) + Math.sqrt(z2);

    return new SphericalCoordinates(phi, theta, projection);
  }

  /**
   * Convert spherical coordinates to cartesian space.
   * 
   * @param coordinates
   */
  public static Translation3d toCartesian(SphericalCoordinates coordinates) {
    double x = coordinates.projection * Math.sin(coordinates.phi) * Math.cos(coordinates.theta);
    double y = coordinates.projection * Math.sin(coordinates.phi) * Math.sin(coordinates.theta);
    double z = coordinates.projection * Math.cos(coordinates.theta);

    return new Translation3d(x, y, z);
  }

}
