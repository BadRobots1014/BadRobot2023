// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the term
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

  private final RuntopositionCommand m_armStoreCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmStoredPos, .2);
  private final RuntopositionCommand m_armHighCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmHighPos, .2);
  private final RuntopositionCommand m_armMediumCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmMediumPos, .2);
  private final RuntopositionCommand m_armLowCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmLowPos, .2);
  private final ArmCommand m_manualPositionCommand;
  private final ArmCommand m_ArmMoveUpCommand = new ArmCommand(m_armSubsystem, .1);
  private final ArmCommand m_ArmMoveDownCommand = new ArmCommand(m_armSubsystem, -.05);
  
  private final GrabberCommandForward m_grabberCommandForward = new GrabberCommandForward(m_grabberSubsystem);
  private final GrabberCommandBackward m_grabberCommandBackward = new GrabberCommandBackward(m_grabberSubsystem);

  private final BalanceCommand m_balancecommand;
  private final DriveStraightCommand m_drivestraightcommand;
  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;

  private final ZeroCommand m_zeroCommand;

  private final DunkCommand m_dunkCommand;

  final static ShuffleboardTab m_tab = Shuffleboard.getTab("Controls");
  
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

  public double getXboxRightY() {
    return Math.abs(xboxController.getRightY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getRightY() : 0;
  }

  public double getXboxLeftY() {
    return Math.abs(xboxController.getLeftY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getLeftY() : 0;
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
    this.m_manualPositionCommand = new ArmCommand(m_armSubsystem, this.getXboxLeftY());

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

    //JoystickButton lightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kBalanceButton);
    //lightButton.whileTrue(this.m_balancecommand);

    // Arm Setting Button Bindings

    // if (xboxController.getBButton()) this.m_armStoreCommand.execute();
    // if (xboxController.getAButton()) this.m_armLowCommand.execute();
    // if (xboxController.getXButton()) this.m_armMediumCommand.execute();
    // if (xboxController.getYButton()) this.m_armHighCommand.execute();

    JoystickButton storeButton = new JoystickButton(this.xboxController, XboxController.Button.kB.value);
    storeButton.toggleOnTrue(m_armStoreCommand);
    m_tab.add("Store Button", "Left Joystick " + XboxController.Button.kB.value);
    JoystickButton lowButton = new JoystickButton(this.xboxController, XboxController.Button.kA.value);
    lowButton.toggleOnTrue(m_armLowCommand);
    m_tab.add("Low Button", XboxController.Button.kA.value);
    JoystickButton mediumButton = new JoystickButton(this.xboxController, XboxController.Button.kX.value);
    mediumButton.toggleOnTrue(m_armMediumCommand);
    m_tab.add("Medium Button", XboxController.Button.kX.value);
    JoystickButton highButton = new JoystickButton(this.xboxController, XboxController.Button.kY.value);
    highButton.toggleOnTrue(m_armHighCommand);
    m_tab.add("High Button", XboxController.Button.kY.value);

    JoystickButton ArmMoveUp = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMoveUp);
    ArmMoveUp.whileTrue(this.m_ArmMoveUpCommand);
    m_tab.add("Move Arm Up", ControllerConstants.kArmMoveUp);

    JoystickButton ArmMoveDown = new JoystickButton(this.leftJoystick, ControllerConstants.kArmMoveDown);
    ArmMoveDown.whileTrue(this.m_ArmMoveDownCommand);
    m_tab.add("Move Arm Down", ControllerConstants.kArmMoveDown);

    // if (xboxController.getBackButton()) this.m_zeroCommand.execute();
    JoystickButton zeroButton = new JoystickButton(this.xboxController, XboxController.Button.kBack.value);
    zeroButton.whileTrue(m_zeroCommand);
    m_tab.add("Zero Button", XboxController.Button.kBack.value);

    // if (xboxController.getAButton()) this.m_dunkCommand.execute();
    JoystickButton dunkButton = new JoystickButton(this.xboxController, XboxController.Button.kStart.value);
    dunkButton.whileTrue(m_dunkCommand);
    m_tab.add("Dunk Button", XboxController.Button.kStart.value);

    // if (xboxController.getRightBumper()) this.m_grabberCommandForward.execute();
    JoystickButton grabForwardButton = new JoystickButton(this.xboxController, XboxController.Button.kLeftBumper.value);
    grabForwardButton.whileTrue(m_grabberCommandForward);
    m_tab.add("Grabber Forward Button", XboxController.Button.kLeftBumper.value);
    // if (xboxController.getLeftBumper()) this.m_grabberCommandBackward.execute();
    JoystickButton grabBackButton = new JoystickButton(this.xboxController, XboxController.Button.kRightBumper.value);
    grabBackButton.whileTrue(m_grabberCommandBackward);
    m_tab.add("Grabber Back Button", XboxController.Button.kRightBumper.value);

    JoystickButton balanceButton = new JoystickButton(this.rightJoystick, ControllerConstants.kBalanceButton);
    balanceButton.whileTrue(this.m_balancecommand);
    m_tab.add("Balance Button", ControllerConstants.kBalanceButton);

    JoystickButton driveStraightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kDriveStraightButton);
    driveStraightButton.whileTrue(this.m_drivestraightcommand);
    driveStraightButton.whileFalse(this.teleopDriveCmd);
    m_tab.add("Drive Straight Button", ControllerConstants.kDriveStraightButton);
   
    
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
