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
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax m_leaderMotor, m_followerMotor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;

    public WristSubsystem() {
        m_leaderMotor = new CANSparkMax(WristConstants.kLeaderMotorID, MotorType.kBrushless);
        m_followerMotor = new CANSparkMax(WristConstants.kFollowerMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(WristConstants.kEncoderID);

        m_encoder.setPositionOffset(WristConstants.kEncoderOffset);

        pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

        pidController.setTolerance(0.03);

        m_leaderMotor.restoreFactoryDefaults();
        m_followerMotor.restoreFactoryDefaults();

        m_leaderMotor.setIdleMode(IdleMode.kCoast);
        m_followerMotor.setIdleMode(IdleMode.kCoast);

        m_leaderMotor.setInverted(true);
        m_followerMotor.setInverted(false);

        m_leaderMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_followerMotor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0.2);

        SmartDashboard.putNumber("angle", 0.1);
    }

    public double getPosistion() {
        double value = -m_encoder.get(); // 反向
        if(value > 1) value -= ((int)value);
        if(value > 0.5) value = 1-value;
        return value;
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wristEncoder", getPosistion());

        double setpoint = SmartDashboard.getNumber("angle", 0.2);
        if(setpoint > 0.231 || setpoint < 0.01) return;
        pidController.setSetpoint(setpoint);

        if(RobotState.isEnabled()) {
            double out = pidController.calculate(getPosistion());
            SmartDashboard.putNumber("out", out);
            m_leaderMotor.set(out);
            m_followerMotor.set(out);
        }
    }

    public Command setPosision(double setpoint) {
        return runOnce(() -> {
            if(setpoint > 0.231 || setpoint < 0.01) return;
            pidController.setSetpoint(setpoint);
        });
    }

    public Command setPosisionBySD() {
        return runOnce(() -> {
            double setpoint = SmartDashboard.getNumber("angle", 0.2);
            if(setpoint > 0.231 || setpoint < 0.01) return;
            pidController.setSetpoint(setpoint);
        });
    }

    public void test(double input) {
        //m_leaderMotor.set(input*0.5);
    }
}
