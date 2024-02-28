package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax m_leftMotor, m_rightMotor;

    private DutyCycleEncoder m_encoder;
    
    private PIDController pidController;
    private double setpoint = 0;
    
    public ElevatorSubsystem() {
        m_leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

        m_encoder = new DutyCycleEncoder(ElevatorConstants.kEncoderID);

        m_encoder.setPositionOffset(ElevatorConstants.kEncoderOffset);

        pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

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

    public double getPosistion() {
        return m_encoder.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("length", getPosistion());

        //m_leftMotor.set(pidController.calculate(this.getPosistion(), setpoint));
        //m_rightMotor.set(pidController.calculate(this.getPosistion(), setpoint));
    }
}
