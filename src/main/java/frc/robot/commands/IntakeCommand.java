package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.DataContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final Timer m_reverseTimer = new Timer();
    private boolean overwrite;

    public IntakeCommand(IntakeSubsystem m_intake, boolean overwrite) {
        this.m_intake = m_intake;
        this.overwrite = overwrite;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(IntakeSubsystem.hasNote() && !overwrite) {
            this.cancel();
            return;
        }
        m_reverseTimer.reset();
        m_intake.setOutput(ConveyorConstants.kIntakeOutput);
    }

    @Override
    public void execute() {
        if(IntakeSubsystem.hasNote() && !overwrite) {
            m_intake.setOutput(-0.15);
            m_reverseTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_reverseTimer.stop();
        m_intake.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !overwrite ? m_reverseTimer.get() > 0.1 && IntakeSubsystem.hasNote() : false;
    }
}
