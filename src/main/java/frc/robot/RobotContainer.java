// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Get triggers (they're analog)
  private static final double TRIGGER_THRESHOLD = 0.3;
  private boolean getLeftTrigger() {
    return this.controller.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
  }
  private boolean getRightTrigger() {
    return this.controller.getRightTriggerAxis() > TRIGGER_THRESHOLD;
  }

  // The controller use to control the robot for this demo
  private final XboxController controller;

  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final ClimberUpCommand climberUpCommand = new ClimberUpCommand(climberSubsystem);
  private final ClimberDownCommand climberDownCommand = new ClimberDownCommand(climberSubsystem);

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShootCommand shootCmd = new ShootCommand(shooterSubsystem, indexerSubsystem, ShooterConstants.farShotPower, "Far Forward");
  private final ShootCommand shootBackCmd = new ShootCommand(shooterSubsystem, indexerSubsystem, ShooterConstants.farBackShotPower, "Far Backward");
  private final ShootCommand closeShootCmd = new ShootCommand(shooterSubsystem, indexerSubsystem, ShooterConstants.closeShotPower, "Close Forward");
  private final ShootCommand closeShootBackCmd = new ShootCommand(shooterSubsystem, indexerSubsystem, ShooterConstants.closeBackShotPower, "Close Backward");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    this.controller = new XboxController(0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Run climbers up when holding the start button
    final JoystickButton climbUpButton = new JoystickButton(this.controller, XboxController.Button.kStart.value);
    climbUpButton.whileHeld(this.climberUpCommand);

    //Run climbers down when holding the select button
    /*Code here*/

    //Shoot in different directions
    Trigger shootTrigger = new Trigger(this::getLeftTrigger);
    shootTrigger.whileActiveContinuous(this.shootCmd);

    Trigger shootBackTrigger = new Trigger(this::getRightTrigger);
    shootBackTrigger.whileActiveContinuous(this.shootBackCmd);

    JoystickButton closeShootBumper = new JoystickButton(this.controller, XboxController.Button.kLeftBumper.value);
    closeShootBumper.whileHeld(this.closeShootCmd);

    JoystickButton closeShootBackBumper = new JoystickButton(this.controller, XboxController.Button.kRightBumper.value);
    closeShootBackBumper.whileHeld(this.closeShootBackCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
