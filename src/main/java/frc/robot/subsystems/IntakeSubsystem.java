package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intake = new CANSparkMax(ConveyorConstants.kIntakeMotorID, MotorType.kBrushless);

    public IntakeSubsystem() {
        m_intake.setInverted(true);
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
        kEmpty("Empty"),
        kTrasmitted("Transmitted"),
        kShooterAccelerating("Shooter Accelerating"),
        kShooterReady("Shooter Ready");

        private String name;

        private Status(String name) {
            this.name = name;
        }

        public String getName() {
            return this.name;
        }
    }

    public Command intakeCommand(boolean reverse) {
        return this.startEnd(
            () -> m_intake.set((reverse ? -1 : 1)*ConveyorConstants.kIntakeOutput),
            () -> m_intake.set(0)
        );
    }
}
