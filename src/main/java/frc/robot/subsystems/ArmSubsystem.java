package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataContainer;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private static final CANSparkMax m_leftMotor = new CANSparkMax(ArmConstants.kLeftMotorID, MotorType.kBrushless), 
        m_rightMotor = new CANSparkMax(ArmConstants.kRightMotorID, MotorType.kBrushless);

    private static final DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderID);
    //private static final Encoder m_relativeEncoder = new Encoder(ArmConstants.kEncoderA, ArmConstants.kEncoderB);
    
    private PIDController pidController;
    private boolean autoShooterAngle = false;

    private double fuxkingOffset = 0;

    public ArmSubsystem() {
        m_encoder.setPositionOffset(ArmConstants.kEncoderOffset);
        pidController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        pidController.setTolerance(0.005);

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
        double value = m_encoder.get(); // 反向
        if(value > 0.5) value = 1-value;
        return -value;
    }

    public void setPosition(double position) {
        if(position > 0.5 || position < 0) return;
        else {
            pidController.setSetpoint(position);
        }
    }

    public Command setPositionCommand(double position) {
        return runOnce(() -> {
            if(autoShooterAngle) setAutoShooterAngle(false);
            setPosition(position);
        });
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public Command resetZero() {
        return runOnce(() -> fuxkingOffset = this.getPosistion());
    }

    public double getHeight() {
        return ArmConstants.kInitialHeight+this.getAdditionHeight();
    }

    public double calculateAngle(double x) {
        return (18*Math.log(x) + 16.666)/360;//(11.477*Math.log(x) + 16.666)/360;
    }

    @Override
    public void periodic() {
        this.updateSmartDashboard();
        if(RobotState.isDisabled()) return;
        if(!m_encoder.isConnected()) {
            stopMotor();
            return;
        }
        if(autoShooterAngle) setPosition(calculateAngle(DataContainer.getDistanceToSpeaker(RobotContainer.getPose().getX(), RobotContainer.getPose().getY())));
        
        double out = pidController.calculate(this.getPosistion());
        m_leftMotor.set(pidController.calculate(this.getPosistion()));
        m_rightMotor.set(out);
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

    public void setAutoShooterAngle(boolean mode) {
        autoShooterAngle = mode;
    }

    public Command toogleAutoShooter() {
        return runOnce(() -> autoShooterAngle = !autoShooterAngle);
    }

    public void updateSmartDashboard() {
        System.out.println("Encoder Value: " + this.getPosistion());
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
