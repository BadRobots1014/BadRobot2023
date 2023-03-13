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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BalancePIDCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DunkCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GrabberCommandBackward;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.commands.GrabberCommandForward;
import frc.robot.commands.RuntopositionCommand;
import frc.robot.commands.ZeroCommand;
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
  private final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final RuntopositionCommand m_armStoreCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmStoredPos, .1);
  private final RuntopositionCommand m_armHighCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmHighPos, .1);
  private final RuntopositionCommand m_armMediumCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmMediumPos, .1);
  private final RuntopositionCommand m_armLowCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmLowPos, .1);
  private final RuntopositionCommand m_manualPositionCommand;
  private final ArmCommand m_ArmMoveUpCommand = new ArmCommand(m_armSubsystem, .1);
  private final ArmCommand m_ArmMoveDownCommand = new ArmCommand(m_armSubsystem, -.05);
  
  private final GrabberCommandForward m_grabberCommandForward = new GrabberCommandForward(m_grabberSubsystem);
  private final GrabberCommandBackward m_grabberCommandBackward = new GrabberCommandBackward(m_grabberSubsystem);

  private final BalanceCommand m_balancecommand;
  private final BalancePIDCommand m_BalancePIDCommand;

  private final DriveStraightCommand m_drivestraightcommand;
  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;

  private final ZeroCommand m_zeroCommand;

  private final DunkCommand m_dunkCommand;

  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  private XboxController xboxController;

  public double getRightY() {
    return Math.abs(rightJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getY() : 0;
  }

  public double getRightZ() {
    return Math.abs(rightJoystick.getZ()) > ControllerConstants.kDeadZoneRadius ? -rightJoystick.getZ() : 0;
  }

  public double getLeftY() {
      return Math.abs(leftJoystick.getY()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getY() : 0;
  }

  public double getLeftZ() {
    return Math.abs(leftJoystick.getZ()) > ControllerConstants.kDeadZoneRadius ? -leftJoystick.getZ() : 0;
  }

  private double getThrottle() {
      return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    this.xboxController = new XboxController(ControllerConstants.kXboxControllerPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);
    this.m_zeroCommand = new ZeroCommand(m_armSubsystem);
    this.m_dunkCommand = new DunkCommand(m_armSubsystem);
    this.m_manualPositionCommand = new RuntopositionCommand(m_armSubsystem, this.getLeftZ(), .04);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, drivetrainSubsystem);
    this.m_drivestraightcommand = new DriveStraightCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, this::getLeftY,this::getRightY, this::getThrottle);
    // this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);   <--- Causes an error right now

    this.m_BalancePIDCommand = new BalancePIDCommand(navxGyroSubsystem, drivetrainSubsystem);
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
      balanceButton.whileTrue(this.m_BalancePIDCommand);
      //balanceButton.whileTrue(this.m_balancecommand);

      JoystickButton driveStraightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kDriveStraightButton);
     
      driveStraightButton.whileTrue(this.m_drivestraightcommand);//drivestraight button
      
      driveStraightButton.whileFalse(this.teleopDriveCmd);
      //balanceButton.whileTrue(this.m_balancecommand);
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
