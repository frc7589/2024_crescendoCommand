package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterLeft = new CANSparkMax(ConveyorConstants.kShooterLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax m_shooterRight = new CANSparkMax(ConveyorConstants.kShooterRightMotorID, MotorType.kBrushless);

    private RelativeEncoder m_shooterLeftEncoder, m_shooterRightEncoder;

    public ShooterSubsystem() {
        m_shooterLeft.setInverted(true);
        m_shooterRight.setInverted(true);

        m_shooterLeftEncoder = m_shooterLeft.getEncoder();
        m_shooterRightEncoder = m_shooterRight.getEncoder();

        SmartDashboard.putNumber("angle", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("speeds", getShooterSpeed());
    }

    public void setSpeed(double output) {
        m_shooterLeft.set(output);
        m_shooterRight.set(output);
    }

    public Command shooterCommand(boolean reverse) {
        return this.startEnd(
            () -> setSpeed((reverse ? -1 : 1)*ConveyorConstants.kShooterOutput),
            () -> setSpeed(0)
        );
    }

    public double[] getShooterSpeed() {
        return new double[] {
            m_shooterLeftEncoder.getVelocity(),
            m_shooterRightEncoder.getVelocity()
        };
    }
}
