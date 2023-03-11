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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmHighCommand;
import frc.robot.commands.ArmLowCommand;
import frc.robot.commands.ArmMediumCommand;
import frc.robot.commands.ArmMoveDownCommand;
import frc.robot.commands.ArmMoveUpCommand;
import frc.robot.commands.ArmStoreCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DunkCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GrabberCommandBackward;
import frc.robot.commands.GrabberCommandForward;
import frc.robot.commands.RuntopositionCommand;
import frc.robot.commands.ZeroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
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

  private final ArmStoreCommand m_armStoreCommand = new ArmStoreCommand(m_armSubsystem);
  private final ArmHighCommand m_armHighCommand = new ArmHighCommand(m_armSubsystem);
  private final ArmMediumCommand m_armMediumCommand = new ArmMediumCommand(m_armSubsystem);
  private final ArmLowCommand m_armLowCommand = new ArmLowCommand(m_armSubsystem);
  private final ArmMoveUpCommand m_ArmMoveUpCommand = new ArmMoveUpCommand(m_armSubsystem);
  private final ArmMoveDownCommand m_ArmMoveDownCommand = new ArmMoveDownCommand(m_armSubsystem);
  
  private final GrabberCommandForward m_grabberCommandForward = new GrabberCommandForward(m_armSubsystem);
  private final GrabberCommandBackward m_grabberCommandBackward = new GrabberCommandBackward(m_armSubsystem);

  private final BalanceCommand m_balancecommand;
  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;


  private ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();

  private final RuntopositionCommand runToPositionCommand;

  private final ZeroCommand m_zeroCommand;

  private final DunkCommand m_dunkCommand;

  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  private XboxController xboxController;

  public double getRightY() {
    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getRightY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getRightY() : 0;
    }
  }

  public double getRightZ() {
    return Math.abs(rightJoystick.getZ()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getZ() : 0;
  }

  public double getLeftY() {
    if (!DriverStation.isJoystickConnected(ControllerConstants.kXboxControllerPort)) {
      return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
    }
    else {
      return Math.abs(xboxController.getLeftY()) > ControllerConstants.kDeadZoneRadius ? -xboxController.getLeftY() : 0;
    }
  }

  public double getLeftZ() {
    return Math.abs(leftJoystick.getZ()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getZ() : 0;
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

    this.runToPositionCommand = new RuntopositionCommand(m_armSubsystem);

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    this.xboxController = new XboxController(ControllerConstants.kXboxControllerPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);
    this.m_zeroCommand = new ZeroCommand(m_armSubsystem);
    this.m_dunkCommand = new DunkCommand(m_armSubsystem);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, m_blinkinSubsystem, drivetrainSubsystem);
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

    //JoystickButton lightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kBalanceButton);
    //lightButton.whileTrue(this.m_balancecommand);

    // Arm Setting Button Bindings

    JoystickButton ArmStoredButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmStoreButton);
    ArmStoredButton.whileTrue(this.m_armStoreCommand);
    
    JoystickButton ArmLowButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmLowButton);
    ArmLowButton.whileTrue(this.m_armLowCommand);
    
    JoystickButton ArmMediumButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMediumButton);
    ArmMediumButton.whileTrue(this.m_armMediumCommand);
    
    JoystickButton ArmHighButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmHighButton);
    ArmHighButton.whileTrue(this.m_armHighCommand);

    JoystickButton ArmMoveUp = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMoveUp);
    ArmMoveUp.whileTrue(this.m_ArmMoveUpCommand);

    JoystickButton ArmMoveDown = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMoveDown);
    ArmMoveDown.whileTrue(this.m_ArmMoveDownCommand);

    JoystickButton ZeroButton = new JoystickButton(this.leftJoystick, ControllerConstants.kArmZeroButton);
    ZeroButton.whileTrue(m_zeroCommand);

    Trigger DunkTrigger = new JoystickButton(this.leftJoystick, ControllerConstants.kDunkTrigger);
    DunkTrigger.whileTrue(m_dunkCommand);

    JoystickButton GrabberForwardButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberFButton);
    GrabberForwardButton.whileTrue(this.m_grabberCommandForward);

    JoystickButton GrabberBackwardButton = new JoystickButton(this.rightJoystick, ControllerConstants.kGrabberRButton);
    GrabberBackwardButton.whileTrue(this.m_grabberCommandBackward);

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
}
