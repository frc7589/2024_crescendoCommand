package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private static DutyCycleEncoder m_encoder;
    private DigitalInput m_switch;
    
    private static PIDController pidController;
    public static boolean notReseted = true;
    
    public ElevatorSubsystem() {
        notReseted = true;
        
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(ElevatorConstants.kEncoderID);

        m_switch = new DigitalInput(ElevatorConstants.kSwitchPortID);

        m_encoder.setPositionOffset(ElevatorConstants.kEncoderOffset);

        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        pidController.setIntegratorRange(0, 0.5);
        pidController.setTolerance(0.03);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);

        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);

        m_leftMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_rightMotor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0.03);

        //m_encoder.reset();

        SmartDashboard.putNumber("setpoint", 0);

        SmartDashboard.putBoolean("reset", false);
    }

    public static double getPosistion() {
        double dis = -m_encoder.getDistance();
        return dis;
    }

    @Override
    public void periodic() {
        if(m_switch.get()) {
            m_encoder.reset();
            if(notReseted) notReseted = false;
        }

        SmartDashboard.putBoolean("switch", m_switch.get());
        SmartDashboard.putBoolean("notReseted", notReseted);

        if(SmartDashboard.getBoolean("reset", false)) {
            m_encoder.reset();
            SmartDashboard.putBoolean("reset", false);
        }

        SmartDashboard.putNumber("length", getPosistion());
        SmartDashboard.putBoolean("conn", m_encoder.isConnected());

        if(WristSubsystem.correctionMode && !notReseted) {
            setPosision(0);
        }
        if(RobotState.isEnabled()) {
            if(notReseted) {
                m_leftMotor.set(-0.3);
                m_rightMotor.set(-0.3);
            } else {
                if(RobotState.isTeleop()) {
                    double out = pidController.calculate(getPosistion());
                    m_leftMotor.set(out);
                    m_rightMotor.set(out);
                }
            }
        }
    }

    public Command setPosisionCommand(double setpoint) {
        return runOnce(() -> {
            if(WristSubsystem.correctionMode) setPosision(setpoint);
        });
    }

    public void setPosision(double setpoint) {
        if(setpoint > 1.82 || setpoint < -0.03) return;
        pidController.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public static double getSetpointPublic() {
        return pidController.getSetpoint();
    }

    public boolean onPoint() {
        return pidController.atSetpoint();
    }


    public void test(double input) {
        m_leftMotor.set(input*0.5);
        m_rightMotor.set(input*0.5);
    }
}
