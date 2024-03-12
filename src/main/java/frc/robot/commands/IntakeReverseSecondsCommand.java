package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.DataContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseSecondsCommand extends Command {
    private final IntakeSubsystem m_intake;

    public IntakeReverseSecondsCommand(IntakeSubsystem m_intake) {
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setOutput(ConveyorConstants.kIntakeOutput);
    }

    @Override
    public void execute() {
        if(m_intake.hasNote()) {
            m_intake.setOutput(-0.3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_intake.hasNote();
    }
}
