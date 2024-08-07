package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WaitingResetCommand extends Command {
    public WaitingResetCommand() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //
    }
    
    @Override
    public void execute() {
        //
    }

    @Override
    public void end(boolean interrupted) {
        //
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !WristSubsystem.correctionMode && !ElevatorSubsystem.notReseted;
    }
}
