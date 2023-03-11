// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Turn15DegreesCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

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
  private final DriveStraightCommand m_drivestraightcommand;
  
  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;
  
  private static Joystick rightJoystick;
  private static Joystick leftJoystick;

  public static JoystickButton turnLeft15Button;
  public static JoystickButton turnRight15Button;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  private XboxController xboxController;

  public static Turn15DegreesCommand m_Turn15DegreesCommand;

  public double getRightY() {
    return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
  }

  public double getLeftY() {
      return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
  }

  private double getThrottle() {
      return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    RobotContainer.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    xboxController = new XboxController(ControllerConstants.kXboxControllerPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);
    m_Turn15DegreesCommand = new Turn15DegreesCommand(navxGyroSubsystem, drivetrainSubsystem, this::getRightY, this::getLeftY);
    
    turnRight15Button = new JoystickButton(rightJoystick, 5);
    turnLeft15Button = new JoystickButton(rightJoystick, 4);


    DrivetrainSubsystem.m_tab.addBoolean("button 4", turnLeft15Button::getAsBoolean);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, drivetrainSubsystem);
    this.m_drivestraightcommand = new DriveStraightCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, this::getLeftY,this::getRightY, this::getThrottle);
    // this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);   <--- Causes an error right now

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
      JoystickButton balanceButton = new JoystickButton(rightJoystick, ControllerConstants.kBalanceButton);
      balanceButton.whileTrue(this.m_balancecommand);
      JoystickButton driveStraightButton = new JoystickButton(leftJoystick, ControllerConstants.kDriveStraightButton);
     turnLeft15Button = new JoystickButton(rightJoystick, 4);
     turnRight15Button = new JoystickButton(rightJoystick, 5);
      driveStraightButton.whileTrue(this.m_drivestraightcommand);//drivestraight button
      
      driveStraightButton.whileFalse(this.teleopDriveCmd);
      balanceButton.whileTrue(this.m_balancecommand);

      if (turnLeft15Button == null)
      {
        System.out.println("Button is null");
      }

      turnLeft15Button.onTrue(m_Turn15DegreesCommand);
      turnRight15Button.onTrue(m_Turn15DegreesCommand);
      System.out.println("Bindings Configured");
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
}
