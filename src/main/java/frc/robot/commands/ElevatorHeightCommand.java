package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElevatorHeightCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double height;
    private final Timer timer = new Timer();
    private final double waitTime;

    public ElevatorHeightCommand(ElevatorSubsystem m_elevator, double height, double waitTime) {
        this.m_elevator = m_elevator;
        this.height = height;
        this.waitTime = waitTime;
        
        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        m_elevator.setPosision(height);
    }

    @Override
    public void execute() {
        if(m_elevator.onPoint()) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= waitTime;
    }
}
