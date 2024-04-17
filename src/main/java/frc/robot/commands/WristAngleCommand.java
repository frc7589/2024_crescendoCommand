package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristAngleCommand extends Command {
    private final WristSubsystem m_wrist;
    private final double angle;
    private final Timer timer = new Timer();
    private final double waitTime;

    public WristAngleCommand(WristSubsystem m_wrist, double angle, double waitTime) {
        this.m_wrist = m_wrist;
        this.angle = angle;
        this.waitTime = waitTime;
        addRequirements(m_wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        m_wrist.setAutoAngle(false);
        m_wrist.setPosision(angle);
    }

    @Override
    public void execute() {
        if(m_wrist.onPoint()) {
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
