package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;

    public ShootingCommand(ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {
        this.m_shooter = m_shooter;
        this.m_intake = m_intake;
        addRequirements(m_shooter, m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(!IntakeSubsystem.hasNote()) {
            this.cancel();
            return;
        }
        m_shooter.setOutput(ConveyorConstants.kShooterOutput);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_shooter.getShooterSpeed()[0] > ConveyorConstants.kShooterSpeed) {
            m_intake.setOutput(ConveyorConstants.kIntakeOutput);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setOutput(0);
        m_intake.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !IntakeSubsystem.hasNote();
    }
}
