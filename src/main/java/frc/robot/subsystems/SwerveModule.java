package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
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

    // private final CANSparkMax driveMotor;
    // private final CANSparkMax turningMotor;

    // private final CANEncoder driveEncoder;
    // private final CANEncoder turningEncoder;

    private final PIDController turningPidController;

    // private final AnalogInput absoluteEncoder;
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
        // absoluteEncoder = new AnalogInput(absoluteEncoderId);
        canCoder = new CANCoder(absoluteEncoderId);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

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

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * 0.0001559;
        // return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        double rad = Math.toRadians(canCoder.getAbsolutePosition()) - absoluteEncoderOffsetRad;

        return rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * 0.0015585245;
        // return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return Math.toRadians(canCoder.getVelocity());
    }

    public double getAbsoluteEncoderRad() {
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        double angle = canCoder.getAbsolutePosition();
        angle *= (2.0 * Math.PI / 180);
        // angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setKp(double input) {
        this.turningPidController.setP(input);
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        double output = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        double setpoint = turningPidController.getSetpoint();
        if (setpoint == 0) {
            setpoint = 0.0001;
        }
        // double thetaInput = (output / setpoint) * 1.1;
        // double thetaInput = (output - setpoint) / (2 * Math.PI);

        // if (thetaInput == Double.POSITIVE_INFINITY || thetaInput ==
        // Double.NEGATIVE_INFINITY) {
        // thetaInput = 0;
        // }

        turningMotor.set(ControlMode.PercentOutput,
                output);
        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Controller out", output);
        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Desired Angle", state.angle.getRadians());
    }

    public void printDebug() {
        SmartDashboard.putNumber("Swerve[" + canCoder.getDeviceID() + "] Current Angle",
                getTurningPosition());
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
