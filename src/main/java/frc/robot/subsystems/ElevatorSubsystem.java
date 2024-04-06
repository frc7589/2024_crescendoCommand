package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor, m_rightMotor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;
    
    public ElevatorSubsystem() {
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

        SmartDashboard.putNumber("kP", 0);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);

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
