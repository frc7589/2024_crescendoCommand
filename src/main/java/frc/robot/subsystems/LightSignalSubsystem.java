package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.Status;

public class LightSignalSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(65);
    private Status oldStatus = null;

    public LightSignalSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    @Override
    public void periodic() {
        Status status = IntakeSubsystem.getStatus();

        if(oldStatus != status) {
            setColor(status.getRed(), status.getGreen(), status.getBlue());
            oldStatus = status;
        }
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if(i < 8) m_ledBuffer.setRGB(i, 0, 0, 0);
            else m_ledBuffer.setRGB(i, ((int)r/2), ((int)g/2), ((int)b/2));
        }
        
        m_led.setData(m_ledBuffer);
    }
}