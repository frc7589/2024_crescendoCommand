package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
        if(WristSubsystem.getSetpointPublic() == Constants.kSendingSetpoints[0] && ElevatorSubsystem.getSetpointPublic() == Constants.kSendingSetpoints[1]) {
            m_shooter.setSetpoint(3600);
        } else {
            m_shooter.setSetpoint(ConveyorConstants.kShooterSpeed);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_shooter.atSetpoint() && Math.abs(m_shooter.getShooterSpeed()[0]-m_shooter.getSetpoint()) < 80) {
            m_intake.setOutput(0.6);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setSetpointCommand(0).schedule();
        if(!interrupted) m_intake.setOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !IntakeSubsystem.hasNote();
    }
}
