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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;

    public WristSubsystem() {
        m_motor = new CANSparkMax(WristConstants.kMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(WristConstants.kEncoderID);

        m_encoder.setPositionOffset(WristConstants.kEncoderOffset);

        pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

        pidController.setTolerance(0.03);

        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setInverted(false);

        m_motor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0.03);
    }

    public double getPosistion() {
        return m_encoder.get() < -0.4 ? (1+(m_encoder.get()%1)) : m_encoder.get()%1;
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wristEncoder", getPosistion());

        if(RobotState.isEnabled()) {
            double out = pidController.calculate(getPosistion());

            m_motor.set(out);
        }
    }

    public Command setPosision(double setpoint) {
        return runOnce(() -> {
            if(setpoint > 0.5 || setpoint < 0) return;
            pidController.setSetpoint(setpoint);
        });
    }

    public Command setPosisionBySD() {
        return runOnce(() -> {
            double setpoint = SmartDashboard.getNumber("angle", 0.2);
            if(setpoint > 0.5 || setpoint < 0) return;
            pidController.setSetpoint(setpoint);
        });
    }

    public void test(double input) {
        m_motor.set(input*0.5);
    }
}
