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
    public static boolean notReseted = true, firstCorrention = true;
    
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

        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setIdleMode(IdleMode.kCoast);

        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_leftMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_rightMotor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0.03);

        //m_encoder.reset();

        SmartDashboard.putNumber("setpoint", 0);

        SmartDashboard.putBoolean("E_reset", false);
    }

    public static double getPosistion() {
        double dis = -m_encoder.getDistance();
        return dis;
    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("switch", m_switch.get());
        SmartDashboard.putBoolean("E_notReseted", notReseted);

        if(SmartDashboard.getBoolean("E_reset", false)) {
            m_encoder.reset();
            SmartDashboard.putBoolean("E_reset", false);
        }

        SmartDashboard.putNumber("length", getPosistion());
        SmartDashboard.putBoolean("conn", m_encoder.isConnected());

        if(WristSubsystem.correctionMode && notReseted) {
            setPosision(0);
        }
        
        if(RobotState.isEnabled()) {
            if(notReseted) {
                if(firstCorrention) {
                    if(m_switch.get()) {
                        SmartDashboard.putString("elevator status", "going up");
                        m_leftMotor.set(0.2);
                        m_rightMotor.set(0.2);
                    } else {
                        firstCorrention = false;
                    }
                } else {
                    if(m_switch.get()) {
                        notReseted = false;
                        m_encoder.reset();
                        SmartDashboard.putString("elevator status", "corrected");
                    } else {
                        m_leftMotor.set(-0.55);
                        m_rightMotor.set(-0.55);
                        SmartDashboard.putString("elevator status", "correcting");
                    }
                }
            } else {
                SmartDashboard.putString("elevator status", "pid controlled");
                double out = pidController.calculate(getPosistion());
                m_leftMotor.set(out);
                m_rightMotor.set(out);
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
