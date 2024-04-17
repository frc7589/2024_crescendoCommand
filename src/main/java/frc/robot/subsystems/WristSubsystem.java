package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax m_leaderMotor, m_followerMotor;

    private static DutyCycleEncoder m_encoder;
    
    private static PIDController pidController;
    private DigitalInput m_switch;
    public static boolean correctionMode = true, firstCorrention = true;
    private double lastPosistion;

    private static double correctedOffset = 0;
    private boolean autoAngle = false;

    public WristSubsystem() {
        m_leaderMotor = new CANSparkMax(WristConstants.kLeaderMotorID, MotorType.kBrushless);
        m_followerMotor = new CANSparkMax(WristConstants.kFollowerMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(WristConstants.kEncoderID);
        m_switch = new DigitalInput(WristConstants.kSwitchPortID);

        pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);

        pidController.setTolerance(0.03);

        m_leaderMotor.restoreFactoryDefaults();
        m_followerMotor.restoreFactoryDefaults();

        m_leaderMotor.setIdleMode(IdleMode.kBrake);
        m_followerMotor.setIdleMode(IdleMode.kBrake);

        m_leaderMotor.setInverted(true);
        m_followerMotor.setInverted(false);

        m_leaderMotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        m_followerMotor.enableVoltageCompensation(Constants.kVoltageCompensation);

        pidController.setSetpoint(0.21);

        lastPosistion = getPosistion();

        SmartDashboard.putNumber("a", 0.0488); // 0.0475
        SmartDashboard.putNumber("k", 0.059);

        SmartDashboard.putData(pidController);
    }

    public static double getPosistion() {
        double value = m_encoder.getAbsolutePosition()-WristConstants.kEncoderOffset-correctedOffset; // 反向
        if(value > 0.5) value = 1-value;
        if(value < 0.1) value = 1+value;
        return value;
    }

    public static double predictAngle(double distance) {
        return SmartDashboard.getNumber("a", 0.0488)*Math.log(distance) + SmartDashboard.getNumber("k", 0.059);
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    public static double getSetpointPublic() {
        return pidController.getSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("wristEncoder", getPosistion());
        SmartDashboard.putNumber("correctedOffset", correctedOffset); 
        SmartDashboard.putBoolean("wristSwitch", m_switch.get());
        SmartDashboard.putNumber("angle_predict", predictAngle(SwerveDriveSubsystem.getDistanceToSpeaker()));

        if((Math.abs(getPosistion()-lastPosistion) > 0.1 || getPosistion() > 0.35) && !correctionMode && !firstCorrention) {
            correctionMode = true;
            firstCorrention = true;
        }

        if(autoAngle) pidController.setSetpoint(predictAngle(SwerveDriveSubsystem.getDistanceToSpeaker()));
        if(RobotState.isEnabled()) {
            if(!ElevatorSubsystem.notReseted) {
                if(correctionMode) {
                    if(ElevatorSubsystem.getPosistion() < 0.05) {
                        if(firstCorrention) {
                            if(m_switch.get()) {
                                m_leaderMotor.set(0.1);
                                m_followerMotor.set(0.1);
                            } else {
                                firstCorrention = false;
                            }
                        } else {
                            if(m_switch.get()) {
                                correctionMode = false;
                                correctedOffset = getPosistion()-0.008;
                                SmartDashboard.putString("status", "corrected");
                            } else {
                                m_leaderMotor.set(-0.1);
                                m_followerMotor.set(-0.1);
                                SmartDashboard.putString("status", "correcting");
                            }
                        }
                    } else {
                        SmartDashboard.putString("status", "Elevator Height Error");
                    }
                } else {
                    SmartDashboard.putString("status", "PID Controlled");
                    double out = pidController.calculate(getPosistion());
                    m_leaderMotor.set(out);
                    m_followerMotor.set(out);
                }
            }
        } else {
            pidController.reset();
        }

        lastPosistion = getPosistion();
    }

    public void setPosision(double setpoint) {
        if(setpoint > 0.231 || setpoint < -0.01) return;
        pidController.setSetpoint(setpoint);
    }

    public Command setPosisionCommand(double setpoint) {
        return runOnce(() -> setPosision(setpoint));
    }

    public void test(double input) {
        //m_leaderMotor.set(input*0.5);
    }

    public boolean onPoint() {
        return pidController.atSetpoint();
    }

    public Command setAutoAngle(boolean enable) {
        return runOnce(() -> {
            autoAngle = enable;
        });
    }
}
