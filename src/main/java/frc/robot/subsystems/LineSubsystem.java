package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LineSubsystem extends SubsystemBase{
    private CANSparkMax m_lineLeftMotor, m_lineRightMotor;


    public LineSubsystem() {
        m_lineLeftMotor = new CANSparkMax(0, MotorType.kBrushless);
        m_lineRightMotor = new CANSparkMax(0, MotorType.kBrushless);

        m_lineLeftMotor.restoreFactoryDefaults();
        m_lineRightMotor.restoreFactoryDefaults();

        m_lineLeftMotor.setIdleMode(IdleMode.kCoast);
        m_lineRightMotor.setIdleMode(IdleMode.kCoast);

        m_lineLeftMotor.setInverted(false);
        m_lineRightMotor.setInverted(true);
    }

    public void set(int speed) {
        m_lineLeftMotor.set(speed);
        m_lineRightMotor.set(speed);
    }
    
}
