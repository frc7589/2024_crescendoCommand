package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;

public class LifterSubsystem extends SubsystemBase{
    private CANSparkMax m_telescope;
    private RelativeEncoder m_encoder;

    public LifterSubsystem() {
        m_telescope = new CANSparkMax(LifterConstants.kMotorID, MotorType.kBrushless);

        m_encoder = m_telescope.getEncoder();
    }

    public void set(double output) {
        m_telescope.set(output*0.8);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Lifter Encoder", m_encoder.getPosition());
    }

}
