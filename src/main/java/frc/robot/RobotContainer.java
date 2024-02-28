// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveryorSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
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
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ConveryorSubsystem m_converyor = new ConveryorSubsystem();
  private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();

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

    m_arm.setPosition(0);
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

    con_util.povUp().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.235), m_arm));
    con_util.povDown().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.003), m_arm));
    con_util.povLeft().onTrue(Commands.runOnce(() -> m_arm.setPosition(0.055), m_arm));

    con_util.y().whileTrue(m_converyor.intakeCommand());

    con_util.axisGreaterThan(Axis.kLeftY.value, 0.2)
      .whileTrue(Commands.runOnce(
          () -> m_arm.setPosition(m_arm.getSetpoint()-con_util.getLeftY()*0.0005),
          m_arm
        ));
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
    m_drive.setPose(pose);
  }
}
