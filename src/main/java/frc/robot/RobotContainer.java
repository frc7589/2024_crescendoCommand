// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.OpzXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();

  private final OpzXboxController con_drive = new OpzXboxController(
    XboxControllerConstants.kDriveControllerID,
    XboxControllerConstants.kControllerMinValue
  );

  private final OpzXboxController con_util =  new OpzXboxController(
    XboxControllerConstants.kUtilControllerID,
    XboxControllerConstants.kControllerMinValue
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(Commands.run(() -> m_drive.drive(
      con_drive.getLeftY(),
      con_drive.getLeftX(),
      con_drive.getRightX()
    ), m_drive));

    //m_elevator.setDefaultCommand(Commands.run(() -> m_elevator.test(con_util.getLeftY()), m_elevator));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    //mController.start().onTrue(Commands.runOnce(() -> {mSwerve.reset()}, mSwerve));

    con_drive.start().onTrue(m_drive.reset());
    con_drive.rightBumper().onTrue(m_drive.increaseMaxOutput());
    con_drive.leftBumper().onTrue(m_drive.decreaseMaxOutput());

    con_drive.leftTrigger(0.5).onTrue(m_drive.setMaxOutput(0.5));
    con_drive.rightTrigger(0.5).onTrue(m_drive.setMaxOutput(0.8));

    //con_util.a().onTrue(Commands.runOnce(() -> m_elevator.setPosision(SmartDashboard.getNumber("ele_height", 0)), m_elevator));
    //con_util.b().onTrue(Commands.runOnce(() -> m_wrist.setPosision(SmartDashboard.getNumber("wrist", 0)), m_wrist));

    con_util.povUp().onTrue(Commands.parallel(
      m_elevator.setPosisionBySD(),
      m_wrist.setPosisionBySD()
    ));
    con_util.povLeft().onTrue(Commands.parallel(
      m_elevator.setPosision(2.0),
      m_wrist.setPosisionBySD()
    ));
    con_util.povDown().onTrue(Commands.parallel(
      m_elevator.setPosision(0),
      m_wrist.setPosision(0.03)
    ));

    con_util.leftBumper().onTrue(m_wrist.setPosision(0.26));

    con_util.a().whileTrue(m_shooter.shooterCommand(true));
    con_util.b().whileTrue(m_shooter.shooterCommand(false));
    con_util.x().whileTrue(m_intake.intakeCommand(true));
    con_util.y().whileTrue(m_intake.intakeCommand(false));

    /* 
    con_util.axisGreaterThan(Axis.kLeftY.value, 0.2).whileTrue(m_elevator.setPosision(m_elevator.getSetpoint()-con_util.getLeftY()*0.0005));
    */

    /*
    con_util.povUp().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.235), m_arm));
    con_util.povDown().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.003), m_arm));
    con_util.povLeft().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.055), m_arm));

    con_util.y().whileTrue(m_converyor.intakeCommand());

    con_util.axisGreaterThan(Axis.kLeftY.value, 0.2)
      .whileTrue(Commands.runOnce(
          () -> m_arm.setPosition(m_arm.getSetpoint()-con_util.getLeftY()*0.0005),
          m_arm
        ));
     */
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.buildAuto("Example Auto");
  }

  public void setPose(Pose2d pose) {
   // m_drive.setPose(pose);
  }
}
