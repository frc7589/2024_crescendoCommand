package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.DataContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeShootingCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final Timer m_reverseTimer = new Timer();

    public IntakeShootingCommand(IntakeSubsystem m_intake) {
        this.m_intake = m_intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(!IntakeSubsystem.hasNote()) {
            this.cancel();
            return;
        }
        m_intake.setOutput(ConveyorConstants.kIntakeOutput);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_reverseTimer.stop();
        m_intake.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !IntakeSubsystem.hasNote();
    }
}
