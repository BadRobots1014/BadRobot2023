// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import frc.robot.util.Logger;

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


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final BalanceCommand m_balancecommand;

  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  private XboxController xboxController;

  private Logger m_logger;

  public double getRightY() {
    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getRightY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getRightY() : 0;
    }
  }

  public double getLeftY() {
    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getLeftY()) > ControllerConstants.kDeadZoneRadius ? -xboxController.getLeftY() : 0;
    }
  }

  private double getThrottle() {
    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
    }
    else {
      return this.xboxController.getBButton() ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    this.xboxController = new XboxController(ControllerConstants.kXboxControllerPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, m_blinkinSubsystem, drivetrainSubsystem);
    // this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);   <--- Causes an error right now

    // Creates the logger
    m_logger = new Logger("/home/lvuser/logs/", "ezlog");

    // Sets the data to
    {
      //DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
      //LocalDateTime now = LocalDateTime.now();

      //m_logger.createStaticField("Time", dtf.format(now));  //DOESN'T WORK
      m_logger.createStaticField("Event", "CORI Preliminaries 1");
      m_logger.recordFrequency(.250);

      m_logger.createDynamicFieldDouble("Motor Speed", drivetrainSubsystem.getMotorSpeed(), drivetrainSubsystem::getMotorSpeed);

      m_logger.setup();
    }
    // NOTE: The logs can be accessed through File Explorer by typing into the bar:
    // ftp://roborio-1014-frc.local/home/lvuser/logs/

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

    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      JoystickButton balanceButton = new JoystickButton(this.rightJoystick, ControllerConstants.kBalanceButton);
      balanceButton.whileTrue(this.m_balancecommand);
    }
    else {
      JoystickButton balanceButton = new JoystickButton(this.xboxController, XboxController.Button.kLeftBumper.value);
      balanceButton.whileTrue(this.m_balancecommand);
    }
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
  
  /**
   * Use this to pass the logger to the main {@link Robot} class to schedule updates.
   *
   * @return the logger
   */
  public Logger getLogger() {
    return m_logger;
  }
}
