package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;
import frc.robot.subsystems.IntakeSubsystem.Status;

public class LightSignalSubsystem extends SubsystemBase{
    private static final AddressableLED m_led = new AddressableLED(0);
    private static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(100);

    public LightSignalSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    @Override
    public void periodic() {
        Status status = IntakeSubsystem.getStatus();
        setColor(status.r, status.g, status.b);
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
         
        m_led.setData(m_ledBuffer);
    }
}