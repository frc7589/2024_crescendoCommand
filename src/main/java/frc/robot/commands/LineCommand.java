package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LineSubsystem;

public class LineCommand extends Command {
    private final LineSubsystem m_LineSubsystem;

    public LineCommand(LineSubsystem m_LineSubsystem, int speed) {
        this.m_LineSubsystem = m_LineSubsystem;
        m_LineSubsystem.set(speed);
    }

}
