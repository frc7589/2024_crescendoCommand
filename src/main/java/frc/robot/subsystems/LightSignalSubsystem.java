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
import frc.robot.Constants.LifterConstants;
import frc.robot.subsystems.IntakeSubsystem.Status;

public class LightSignalSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(1);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(100);
    private int oldR = 0, oldG = 0, oldB = 0;
    private Timer blinkingTimer = new Timer();

    public LightSignalSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    @Override
    public void periodic() {
        Status status = IntakeSubsystem.getStatus();
        setColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void setColor(int r, int g, int b) {
        if(oldR != r || oldG != g || oldB != b) {
            blinkingTimer.start();
            oldR = r;
            oldG = g;
            oldB = b;
            return;
        }

        if(blinkingTimer.get() >= 1.5) {
            blinkingTimer.stop();
            blinkingTimer.reset();
            return;
        }

        if(blinkingTimer.get()%0.1 < 0.05 && blinkingTimer.get() != 0) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, ((int)r/10), ((int)g/10), ((int)b/10));
            }
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, ((int)r/2), ((int)g/2), ((int)b/2));
            }
        }
        
        m_led.setData(m_ledBuffer);
    }
}