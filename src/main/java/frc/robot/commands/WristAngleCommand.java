package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class WristAngleCommand extends Command {
    private final WristSubsystem m_wrist;
    private double angle;
    private final Timer timer = new Timer();
    private double waitTime;
    private boolean started = false;

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
        m_wrist.setAutoAngleNC(false);
        m_wrist.setPosision(angle);
        this.started = false;
        System.out.println("run angle" + angle);

        if(angle > 0.2) {
            RobotContainer.getShooterSubsystem().setSetpoint(0);
        }
    }

    @Override
    public void execute() {
        if(m_wrist.onPoint()) {
            System.out.println("reach angle" + angle);
            if(!started) {
                timer.start();
                this.started = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("end angle" + angle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= waitTime;
    }
}
