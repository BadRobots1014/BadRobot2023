// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.BlinkinCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final BlinkinSubsystem m_blinkinSubsystem = new BlinkinSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final BlinkinCommand m_blinkinCommand = new BlinkinCommand(m_blinkinSubsystem);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private XboxController xboxController;

  public double getRightY() {
    if (xboxController == null) {
      return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getRightY()) > ControllerConstants.kDeadZoneRadius ? -xboxController.getRightY() : 0;
    }
  }

  public double getLeftY() {
    if (xboxController == null) {
      return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getLeftY()) > ControllerConstants.kDeadZoneRadius ? -xboxController.getLeftY() : 0;
    }
  }

  private double getThrottle() {
    return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    this.xboxController = new XboxController(Constants.XBoxConstants.XBoxPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);

    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton lightButton = new JoystickButton(this.leftJoystick, 1);
    lightButton.whileTrue(this.m_blinkinCommand);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
