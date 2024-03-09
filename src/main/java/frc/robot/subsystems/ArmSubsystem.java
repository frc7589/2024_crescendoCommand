package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataContainer;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor, m_rightMotor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;
    private boolean autoShooterAngle = false;

    public ArmSubsystem() {
        m_leftMotor = new CANSparkMax(ArmConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ArmConstants.kRightMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderID);

        m_encoder.setPositionOffset(ArmConstants.kEncoderOffset);

        pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        pidController.setTolerance(0.03);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_leftMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_rightMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
    }

    public double getAdditionHeight() {
        return ArmConstants.kArmLength*Math.sin(this.getRadians());
    }

    public double getDegrees() {
        return this.getPosistion()*360;
    }

    public double getRadians() {
        return this.getPosistion()*2*Math.PI;
    }

    public double getPosistion() {
        double v = -m_encoder.get();
        return (v < -0.4 ? (1+(v%1)) : v%1);
    }

    public void setPosition(double position) {
        if(position > 0.31 || position < 0) return;
        else {
            pidController.setSetpoint(position);
        }
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public void setAngleForSpeaker(double distance) {
        this.setPosition((11.477*Math.log(distance) + 16.666)/360);
    }

    public double getHeight() {
        return ArmConstants.kInitialHeight+this.getAdditionHeight();
    }

    public double calculateAngle(double x) {
        return (11.477*Math.log(x) + 16.666)/360;
    }

    @Override
    public void periodic() {
        this.updateSmartDashboard();
        if(RobotState.isDisabled()) return;
        if(autoShooterAngle) setPosition(calculateAngle(DataContainer.getDistanceToSpeaker(DataContainer.pose2d.getX(), DataContainer.pose2d.getY())));
        m_leftMotor.set(pidController.calculate(this.getPosistion()));
        m_rightMotor.set(pidController.calculate(this.getPosistion()));
    }

    public boolean onPoint() {
        return pidController.atSetpoint();
    }

    public void stopMotor() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    public void updatePID(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    public double getShooterAngle(double armAngle) {
        return ArmConstants.kInitialAngle - armAngle;
    }

    public Command toogleAutoShooter() {
        return runOnce(() -> autoShooterAngle = !autoShooterAngle);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("[Arm] Current Height", ArmConstants.kInitialHeight+this.getAdditionHeight());
        SmartDashboard.putNumber("[Arm] Current Angle", this.getDegrees());
        SmartDashboard.putNumber("[Arm] Setpoint Angle", this.getSetpoint()*360);
        SmartDashboard.putNumber("[Arm] Encoder Value", this.getPosistion());
        SmartDashboard.putNumber("[Arm] Shooter Angle", getShooterAngle(this.getDegrees()));
        SmartDashboard.putBoolean("[Arm] Encoder Connection", m_encoder.isConnected());
        SmartDashboard.putBoolean("[Arm] On Setpoint", pidController.atSetpoint());

        SmartDashboard.putBoolean("[Arm] Auto Angle", this.autoShooterAngle);

        SmartDashboard.putNumber("[Arm] Output", m_leftMotor.get());
    }
}
