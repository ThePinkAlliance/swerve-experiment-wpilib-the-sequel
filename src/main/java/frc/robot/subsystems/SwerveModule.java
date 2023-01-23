package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder canCoder;
    private final TalonFXConfiguration steerConfig;
    private final TalonFXConfiguration driveConfig;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningEncoderReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.steerConfig = new TalonFXConfiguration();
        this.driveConfig = new TalonFXConfiguration();
        canCoder = new CANCoder(absoluteEncoderId);

        // Changed from 360 to 180 no change in pod behvior was noticed.
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        turningMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.configAllSettings(driveConfig);
        turningMotor.configAllSettings(steerConfig);

        driveMotor.setInverted(driveMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0.5, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    /**
     * Returns the drive wheel position in meters.
     */
    public double getDrivePosition() {
        return ((driveMotor.getSelectedSensorPosition() / 2048.0) * Constants.ModuleConstants.kDriveMotorGearRatio)
                * Constants.ModuleConstants.kWheelDiameterMeters * Math.PI;
    }

    /*
     * This returns the current position of the steer shaft in radians.
     */
    public double getTurningPosition() {
        double rad = Math.toRadians(canCoder.getAbsolutePosition()) - absoluteEncoderOffsetRad;

        return rad;
    }

    public double getDriveVelocity() {
        /*
         * It might be necessary to change the constant because it does not take into
         * account the gear ratio.
         */
        return driveMotor.getSelectedSensorVelocity() * 0.0015585245;
    }

    /**
     * Returns the velocity of the steer motor in rad/sec.
     */
    public double getTurningVelocity() {
        return Math.toRadians(canCoder.getVelocity());
    }

    @Deprecated
    public double getAbsoluteEncoderRad() {
        double angle = canCoder.getAbsolutePosition();
        angle *= (2.0 * Math.PI / 180);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    /*
     * Returns the current state of the swerve module using velocity.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /*
     * Returns the current state of the swerve module using position.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setKp(double input) {
        this.turningPidController.setP(input);
    }

    /**
     * Sets the current module state to the desired one.
     * 
     * @param state desired swerve module state
     */
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        double output = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(ControlMode.PercentOutput,
                output);

        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Controller out", output);
        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Desired Angle", state.angle.getRadians());
    }

    public void printDebug() {
        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Current Angle",
                getTurningPosition());

        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Current Distance",
                getPosition().distanceMeters);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
