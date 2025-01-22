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

        m_leaderMotor.enableVoltageCompensation(8);
        m_followerMotor.enableVoltageCompensation(8);

        pidController.setSetpoint(0.21);

        lastPosistion = getPosistion();

        /* 
        SmartDashboard.putNumber("a", 0.0488); // 0.0475
        SmartDashboard.putNumber("k", 0.057);
        */

        SmartDashboard.putNumber("a", WristConstants.autoShootingParams[0]);
        SmartDashboard.putNumber("b", WristConstants.autoShootingParams[1]);
        SmartDashboard.putNumber("c", WristConstants.autoShootingParams[2]);
        SmartDashboard.putNumber("d", WristConstants.autoShootingParams[3]);
        SmartDashboard.putNumber("e", WristConstants.autoShootingParams[4]);
        SmartDashboard.putData(pidController);
    }

    public static double getAbsolutePosition() {
        return m_encoder.getAbsolutePosition();
    }

    public static double getPosistion() {
        double value = m_encoder.getAbsolutePosition()-WristConstants.kEncoderOffset-correctedOffset; // 反向
        if(value > 0.5) value = 1-value;
        if(value < -0.1) value = -value;
        return value;
    }

    public static double getCorrectedPosistion() {
        double value = m_encoder.getAbsolutePosition()-WristConstants.kEncoderOffset-correctedOffset; // 反向
        return value;
    }

    public static double predictAngle(double distance) {
        //if(RobotState.isTest()) {
        return (
            SmartDashboard.getNumber(
                "a",
                WristConstants.autoShootingParams[0])*Math.pow(distance, 4)+
            SmartDashboard.getNumber("b", WristConstants.autoShootingParams[1])*Math.pow(distance, 3)+
            SmartDashboard.getNumber("c", WristConstants.autoShootingParams[2])*Math.pow(distance, 2)+
            SmartDashboard.getNumber("d", WristConstants.autoShootingParams[3])*distance+
            SmartDashboard.getNumber("e", WristConstants.autoShootingParams[4])
        );
        //}
        // return SmartDashboard.getNumber("a", 0.0488)*Math.log(distance) + SmartDashboard.getNumber("k", 0.057);
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
        SmartDashboard.putNumber("W_correctedOffset", correctedOffset); 
        SmartDashboard.putNumber("a_wristEncoder", getCorrectedPosistion());
        SmartDashboard.putBoolean("wristSwitch", m_switch.get());
        SmartDashboard.putNumber("angle_predict", predictAngle(SwerveDriveSubsystem.getDistanceToSpeaker()));
        SmartDashboard.putBoolean("W_firstCorrention", firstCorrention);
        SmartDashboard.putBoolean("W_correctionMode", correctionMode);
        SmartDashboard.putNumber("Wrist Absolute Position", getAbsolutePosition());

        SmartDashboard.putBoolean("autoAngle", autoAngle);

        if((Math.abs(getPosistion()-lastPosistion) > 0.2 || getPosistion() > 0.35) && !correctionMode && !firstCorrention) {
            correctionMode = true;
            firstCorrention = true;
            SmartDashboard.putNumberArray("lastError", new Double[]{getPosistion(), getCorrectedPosistion()});
        }

        if(autoAngle) setPosision(predictAngle(SwerveDriveSubsystem.getDistanceToSpeaker()));
        
        if(RobotState.isEnabled()) {
            if(!ElevatorSubsystem.notReseted) {
                if(correctionMode) {
                    pidController.reset();
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
                                correctedOffset = getPosistion()-0.045;
                                SmartDashboard.putString("status", "corrected");
                            } else {
                                m_leaderMotor.set(-0.12);
                                m_followerMotor.set(-0.12);
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
        if(setpoint > 0.252 || setpoint < -0.01) return;
        pidController.setSetpoint(setpoint);
    }

    public Command setPosisionCommand(double setpoint) {
        return this.runOnce(() -> setPosision(setpoint));
    }

    public void test(double input) {
        //m_leaderMotor.set(input*0.5);
    }

    public boolean onPoint() {
        return pidController.atSetpoint();
    }

    public Command setAutoAngle(boolean enable) {
        return this.runOnce(() -> {
            autoAngle = enable;
        });
    }
    public void setAutoAngleNC(boolean enable) {
        autoAngle = enable;
    }
}
