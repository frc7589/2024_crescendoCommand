package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax m_shooterLeft = new CANSparkMax(ConveyorConstants.kShooterLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax m_shooterRight = new CANSparkMax(ConveyorConstants.kShooterRightMotorID, MotorType.kBrushless);

    private RelativeEncoder m_shooterLeftEncoder, m_shooterRightEncoder;

    private PIDController pidController;

    private boolean usingPID = true;

    public ShooterSubsystem() {

        m_shooterLeft.setIdleMode(IdleMode.kCoast);
        m_shooterRight.setIdleMode(IdleMode.kCoast);

        m_shooterLeft.setInverted(true);
        m_shooterRight.setInverted(true);

        m_shooterLeft.enableVoltageCompensation(8);
        m_shooterRight.enableVoltageCompensation(8);

        pidController = new PIDController(0.00013, 0.0009, 0);

        pidController.setTolerance(50);

        m_shooterLeftEncoder = m_shooterLeft.getEncoder();
        m_shooterRightEncoder = m_shooterRight.getEncoder();

        SmartDashboard.putData("shooterPID", pidController);

        pidController.setSetpoint(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("speeds", getShooterSpeed());

        if(RobotState.isEnabled() && usingPID) {
            setOutput(pidController.calculate((getShooterSpeed()[0]+getShooterSpeed()[1])/2));
        }
    }

    public void setOutput(double output) {
        m_shooterLeft.set(output);
        m_shooterRight.set(output);
    }

    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public Command shooterCommand() {
        return this.startEnd(
            () -> pidController.setSetpoint(ConveyorConstants.kShooterSpeed),
            () -> pidController.setSetpoint(1400)
        );
    }

    public Command setSetpointCommand(double setpoint) {
        return runOnce(() -> setSetpoint(setpoint));
    }

    public Command shooterReverseCommand() {
        return this.startEnd(
            () -> {
                usingPID = false;
                setOutput(-0.3);
            },
            () -> {
                usingPID = true;
            }
        );
    }


    public double[] getShooterSpeed() {
        return new double[] {
            m_shooterLeftEncoder.getVelocity(),
            m_shooterRightEncoder.getVelocity()
        };
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }
}
