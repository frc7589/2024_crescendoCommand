package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ConveyorConstants;

public class ConveryorSubsystem extends PIDSubsystem {
    private final CANSparkMax m_intake = new CANSparkMax(ConveyorConstants.kIntakeMotorID, MotorType.kBrushless);
    private final CANSparkMax m_shooterLeft = new CANSparkMax(ConveyorConstants.kShooterLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax m_shooterRight = new CANSparkMax(ConveyorConstants.kShooterRightMotorID, MotorType.kBrushless);

    private RelativeEncoder m_shooterLeftEncoder, m_shooterRightEncoder;

    private final SimpleMotorFeedforward m_shooterFeedforward =
          new SimpleMotorFeedforward(
            0, 0);

    public ConveryorSubsystem() {
        super(new PIDController(0, 0, 0));

        getController().setTolerance(100);

        setSetpoint(ConveyorConstants.kShooterSpeed);

        m_intake.setInverted(true);

        m_shooterLeft.setInverted(true);
        m_shooterRight.setInverted(true);

        m_shooterLeftEncoder = m_shooterLeft.getEncoder();
        m_shooterRightEncoder = m_shooterRight.getEncoder();
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

    @Override
    protected void useOutput(double output, double setpoint) {
        m_shooterLeft.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
        m_shooterLeft.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    }

    @Override
    protected double getMeasurement() {
        return (m_shooterLeftEncoder.getVelocity() + m_shooterRightEncoder.getVelocity())/2;
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }

    public Command intakeCommand() {
        return this.startEnd(
            
            () -> m_intake.set(ConveyorConstants.kIntakeOutput),
            () -> m_intake.set(0)
        );
    }
}
