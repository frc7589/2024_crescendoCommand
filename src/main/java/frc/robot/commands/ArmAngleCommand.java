package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ArmAngleCommand extends Command {
    private final ArmSubsystem m_arm;
    private final double angle;
    private final Timer timer = new Timer();

    public ArmAngleCommand(ArmSubsystem m_arm, double angle) {
        this.m_arm = m_arm;
        this.angle = angle;
        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        m_arm.setAutoShooterAngle(false);
        m_arm.setPosition(angle);
    }

    @Override
    public void execute() {
        if(m_arm.onPoint()) {
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
        return timer.get() >= 0.8;
    }
}
