package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static final CANSparkMax m_intake = new CANSparkMax(ConveyorConstants.kIntakeMotorID, MotorType.kBrushless);
    private static final DigitalInput m_sensor = new DigitalInput(ConveyorConstants.kSensorPortID);
    private static Status status = Status.kEmpty;

    public IntakeSubsystem() {
        m_intake.setInverted(false);
    }

    public static enum Mode {
        kAuxiliary("Auxiliary"),
        kManual("Manual");

        private String name;

        private Mode(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }

    public static enum Status {
        kEmpty("Empty", 255, 238, 0),
        kTrasmitted("Tramsmitted", 132, 0, 255);

        private String name;
        private int red, green, blue;

        private Status(String name, int r, int g, int b) {
            this.name = name;
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        public String getName() {
            return this.name;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }
    }

    @Override
    public void periodic() {
        if(hasNote()) status = Status.kTrasmitted;
        else status = Status.kEmpty;
    }

    public static Status getStatus() {
        return status;
    }

    public Command intakeCommand(boolean reverse) {
        double speed = reverse ? -0.2 : ConveyorConstants.kIntakeOutput;
        return this.startEnd(
            () -> m_intake.set(speed),
            () -> m_intake.set(0)
        );
    }

    public void setOutput(double output) {
        m_intake.set(output);
    }

    public double get() {
        return m_intake.get();
    }

    public static boolean hasNote() {
        return m_sensor.get();
    }
}
