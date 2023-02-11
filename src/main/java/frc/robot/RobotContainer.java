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
import frc.robot.commands.DriverPresetNextCommand;
import frc.robot.commands.DriverPresetPreviousCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ColorSensorCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriverPresetsSubsystem;
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
  private final DriverPresetsSubsystem m_DriverPresetsSubsystem = new DriverPresetsSubsystem();

  private final BlinkinCommand m_blinkinCommand = new BlinkinCommand(m_blinkinSubsystem);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriverPresetNextCommand m_DriverPresetNextCommand = new DriverPresetNextCommand(m_DriverPresetsSubsystem);
  private final DriverPresetPreviousCommand m_DriverPresetPreviousCommand = new DriverPresetPreviousCommand(m_DriverPresetsSubsystem);


  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;

  private ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();

  private ColorSensorCommand colorSensorCommand = new ColorSensorCommand(colorSensorSubsystem);
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  public double getRightY() {
    return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
  }

  public double getLeftY() {
    return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
  }

  private double getThrottle() {
    return this.rightJoystick.getRawButton(DriverPresetsSubsystem.getkThrottleButton()) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);

    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);

    this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);

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
    JoystickButton Button6 = new JoystickButton(leftJoystick, 6);
    Button6.whenReleased(m_DriverPresetNextCommand);
    JoystickButton Button7 = new JoystickButton(leftJoystick, 6);
    Button7.whenReleased(m_DriverPresetPreviousCommand);
    
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
