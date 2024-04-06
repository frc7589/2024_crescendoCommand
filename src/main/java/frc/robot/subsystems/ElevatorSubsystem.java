package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor, m_rightMotor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;

    private XboxController controller;
    
    public ElevatorSubsystem() {
        controller = new XboxController(0);

        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(ElevatorConstants.kEncoderID);

        m_encoder.setPositionOffset(ElevatorConstants.kEncoderOffset);

        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        pidController.setIntegratorRange(0, 0.5);
        pidController.setTolerance(0.03);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setIdleMode(IdleMode.kCoast);

        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_leftMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_rightMotor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0);

        m_encoder.reset();

        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);

        SmartDashboard.putNumber("setpoint", 0);

        SmartDashboard.putNumber("ele", 0.0);

        SmartDashboard.putBoolean("reset", false);
    }

    public double getPosistion() {
        return -m_encoder.getDistance();
    }

    @Override
    public void periodic() {

        if(SmartDashboard.getBoolean("reset", false)) {
            m_encoder.reset();
            SmartDashboard.putBoolean("reset", false);
        }
        SmartDashboard.putNumber("length", this.getPosistion());
        SmartDashboard.putBoolean("conn", m_encoder.isConnected());

        if(RobotState.isEnabled()) {
            double out = pidController.calculate(this.getPosistion());
            m_leftMotor.set(out);
            m_rightMotor.set(out);
        }

        pidController.setPID(
            SmartDashboard.getNumber("kP", 0),
            SmartDashboard.getNumber("kI", 0),
            SmartDashboard.getNumber("kD", 0)
        );

        if(m_encoder.getDistance() < 0.1 || m_encoder.getDistance() > 1.8) {
            return;
        }

        double setpoint = SmartDashboard.getNumber("setpoint", 0);

        if(setpoint > 1.95 || setpoint < 0) return;
        
        pidController.setSetpoint(setpoint);

        m_leftMotor.set(pidController.calculate(m_encoder.getDistance(), setpoint));
        m_rightMotor.set(pidController.calculate(m_encoder.getDistance(), setpoint));

        if(controller.getLeftY() > 0.1) {
            m_leftMotor.set(0.3);
            m_rightMotor.set(0.3);
        } else if(controller.getLeftY() < -0.1) {
            m_leftMotor.set(-0.3);
            m_rightMotor.set(-0.3);
        } else {
            m_leftMotor.set(0);
            m_rightMotor.set(0);
        }

    }

    public Command setPosision(double setpoint) {
        return runOnce(() -> {
            if(setpoint > 2.45 || setpoint < -0.05) return;
            pidController.setSetpoint(setpoint);
        });
    }


    public Command setPosisionBySD() {
        return runOnce(() -> {
            double setpoint = SmartDashboard.getNumber("ele", 2.0);
            if(setpoint > 2.45 || setpoint < -0.05) return;
            pidController.setSetpoint(setpoint);
        });
    }

    

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public void test(double input) {
        m_leftMotor.set(input*0.5);
        m_rightMotor.set(input*0.5);
    }
}
