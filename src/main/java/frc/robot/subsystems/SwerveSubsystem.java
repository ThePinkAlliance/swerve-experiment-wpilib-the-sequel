package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry estimator = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            getRotation2d(), new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            }, new Pose2d());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public List<SwerveModulePosition> getPositions() {
        return List.of(frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * NOTE: This method has been returning incorrect positions when using
     * SwerveControllerCommand
     */
    public Pose2d getPose() {
        return estimator.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        estimator.resetPosition(getRotation2d(),
                new SwerveModulePosition[] { frontLeft
                        .getPosition(),
                        frontRight
                                .getPosition(),
                        backLeft.getPosition(),
                        backRight
                                .getPosition() },
                pose);
    }

    /*
     * Set the heading of all the swerve pods.
     */
    public void setAllHeadings(double heading) {
        frontLeft.setDesiredState(
                new SwerveModuleState(frontLeft.getState().speedMetersPerSecond, new Rotation2d(heading)));
        frontRight.setDesiredState(
                new SwerveModuleState(frontRight.getState().speedMetersPerSecond, new Rotation2d(heading)));
        backLeft.setDesiredState(
                new SwerveModuleState(backLeft.getState().speedMetersPerSecond, new Rotation2d(heading)));
        backRight.setDesiredState(
                new SwerveModuleState(backRight.getState().speedMetersPerSecond, new Rotation2d(heading)));
    }

    public void setKp(double kP) {
        frontLeft.setKp(kP);
        frontRight.setKp(kP);
        backLeft.setKp(kP);
        backRight.setKp(kP);
    }

    @Override
    public void periodic() {
        estimator.update(getRotation2d(),
                new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                        backRight.getPosition() });

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Location X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Robot Location Y", getPose().getTranslation().getY());

        frontLeft.printDebug();
        frontRight.printDebug();
        backLeft.printDebug();
        backRight.printDebug();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
