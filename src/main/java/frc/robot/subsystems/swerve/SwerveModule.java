package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConfig;

/** Swerve模組 Class */
public class SwerveModule {
    private CANSparkMax m_throttle, m_rotor;

    private CANcoder m_rotorEncoder;
    private RelativeEncoder m_throttleEncoder;

    private PIDController pid_rotor;

    /**
     * Swerve模組
     * @param config Swerve模組配置
     */
    public SwerveModule(SwerveModuleConfig config) {
        m_throttle = new CANSparkMax(config.getThrottleID(), MotorType.kBrushless);
        m_throttleEncoder = m_throttle.getEncoder();
        m_rotor = new CANSparkMax(config.getRotorID(), MotorType.kBrushless);
        m_rotorEncoder = new CANcoder(config.getRotorEncoderId());

        m_throttle.restoreFactoryDefaults();
        m_rotor.restoreFactoryDefaults();
        m_rotorEncoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(config.getRotorAngleOffset())
                .withSensorDirection(SwerveDriveConstants.kRotorEncoderDirection)
        );

        m_throttle.setInverted(SwerveDriveConstants.kThrottleMotorInversion);
        m_rotor.setInverted(SwerveDriveConstants.kRotorMotorInversion);
        m_rotor.enableVoltageCompensation(Constants.kVoltageCompensation*0.6);
        m_rotor.setIdleMode(IdleMode.kBrake);

        m_throttle.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_throttle.setIdleMode(IdleMode.kBrake);

        m_throttleEncoder.setVelocityConversionFactor(SwerveDriveConstants.kThrottleVelocityConversionFactor);
        m_throttleEncoder.setPositionConversionFactor(SwerveDriveConstants.kThrottlePositionConversionFactor);

        pid_rotor = new PIDController(
            SwerveDriveConstants.kRotor_kP,
            SwerveDriveConstants.kRotor_kI,
            SwerveDriveConstants.kRotor_kD
        );
        pid_rotor.enableContinuousInput(-180, 180);
    }

    /**
     * 取得動力馬達實例
     * @return 動力馬達實例
     */
    public CANSparkMax getThrottleMotor() {
        return m_throttle;
    }

    /**
     * 取得轉向馬達實例
     * @return 轉向馬達實例
     */
    public CANSparkMax getRotorMotor() {
        return m_rotor;
    }

    /**
     * Only available in TEST mode
     * update rotor's PID Controller coefficients
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     * @return is PID coefficients updated
     */
    public boolean updateRotorPID(double kp, double ki, double kd) {
        if(!RobotState.isTest()) return false;

        pid_rotor.setPID(kp, ki, kd);

        return (
            pid_rotor.getP() == kp && 
            pid_rotor.getI() == ki && 
            pid_rotor.getD() == kd
        );
    }

    /**
     * get current rotor's PID Controller coefficients
     * @return rotor's PID Controller coefficients
     */
    public double[] getRotorPID() {
        return new double[] {
            pid_rotor.getP(),
            pid_rotor.getI(),
            pid_rotor.getD()
        };
    }

    /**
     * get current speed of throttle motor
     * @return throttle motor speed (unit: m/s)
     */
    public double getSpeed() {
        return m_throttleEncoder.getVelocity();
    }

    /**
     * get Swerve Module State
     * @return SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_throttleEncoder.getVelocity(),
            Rotation2d.fromRotations(m_rotorEncoder.getAbsolutePosition().refresh().getValue())
        );
    }

    /**
     * get Swerve Module Position
     * @return SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            -m_throttleEncoder.getPosition(),
            Rotation2d.fromRotations(m_rotorEncoder.getAbsolutePosition().refresh().getValue())
        );
    }

    /**
     * set Swerve Module State
     * @param SwerveModuleState state
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState opzState = SwerveModuleState.optimize(state, getState().angle);

        m_rotor.set(
            pid_rotor.calculate(
                getState().angle.getDegrees(),
                opzState.angle.getDegrees()
            )
        );
    
        m_throttle.set(opzState.speedMetersPerSecond);
    }

    /**
     * 測試執行
     * @param throttle 動力馬達輸出
     * @param rotor    轉向馬達輸出
     */
    public void testRunning(double throttle, double rotor) {
        m_throttle.set(throttle);
        m_rotor.set(rotor);
    }

    public void reset() {
        pid_rotor.reset();
    }
}