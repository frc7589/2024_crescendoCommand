package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.DataContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootingAngleCommand extends Command {
    private final ArmSubsystem m_arm;

    public AutoShootingAngleCommand(ArmSubsystem m_arm) {
        this.m_arm = m_arm;
        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.setPosition(m_arm.calculateAngle(DataContainer.getDistanceToSpeaker(DataContainer.camPose.getX(), DataContainer.camPose.getY())));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_arm.onPoint();
    }
}
