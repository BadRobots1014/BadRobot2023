// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmHighCommand;
import frc.robot.commands.ArmLowCommand;
import frc.robot.commands.ArmMediumCommand;
import frc.robot.commands.ArmStoreCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ColorSensorCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
  private final ArmStoreCommand m_armStoreCommand = new ArmStoreCommand(m_armSubsystem);
  private final ArmHighCommand m_armHighCommand = new ArmHighCommand(m_armSubsystem);
  private final ArmMediumCommand m_armMediumCommand = new ArmMediumCommand(m_armSubsystem);
  private final ArmLowCommand m_armLowCommand = new ArmLowCommand(m_armSubsystem);
  private final GrabberCommand m_grabberCommand;
  private final BalanceCommand m_balancecommand;
  private DriveCommand teleopDriveCmd;
  private String runMode;

  private DrivetrainSubsystem drivetrainSubsystem;

  private ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();

  private ColorSensorCommand colorSensorCommand = new ColorSensorCommand(colorSensorSubsystem);
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  public double getRightY() {
    return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
  }

  public double getLeftY() {
    return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
  }

  private double getThrottle() {
    return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  public double getRightZ() {
    return Math.abs(rightJoystick.getZ()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getZ() : 0;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, m_blinkinSubsystem, drivetrainSubsystem);
    this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);

    this. m_grabberCommand = new GrabberCommand(m_armSubsystem, runMode, this::getRightZ); // In Progress - 

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
    JoystickButton lightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kBalanceButton);
    lightButton.whileTrue(this.m_balancecommand);

    // Arm Setting Button Bindings

    JoystickButton ArmStoredButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmStoreButton);
    ArmStoredButton.whileTrue(this.m_armStoreCommand);
    
    JoystickButton ArmLowButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmLowButton);
    ArmLowButton.whileTrue(this.m_armLowCommand);
    
    JoystickButton ArmMediumButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMediumButton);
    ArmMediumButton.whileTrue(this.m_armMediumCommand);
    
    Trigger ArmHighButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmHighButton);
    ArmHighButton.whileTrue(this.m_armHighCommand);

    JoystickButton GrabberManFButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberManFButton);
    if(GrabberManFButton.getAsBoolean()){
      runMode = ArmConstants.kManualRunForward;
    }

    JoystickButton GrabberManRButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberManRButton);
    if(GrabberManRButton.getAsBoolean()){
      runMode = ArmConstants.kManualRunBackward;
    }

    JoystickButton GrabberPresetFButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberPresetFButton);
    if(GrabberPresetFButton.getAsBoolean()){
      runMode = ArmConstants.kPresetRunForward;
    }

    JoystickButton GrabberPresetRButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberPresetRButton);
    if(GrabberPresetRButton.getAsBoolean()){
      runMode = ArmConstants.kPresetRunBackward;
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
