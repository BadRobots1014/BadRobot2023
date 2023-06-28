// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
import frc.robot.commands.ManualCommand;
import frc.robot.commands.RuntopositionCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.ZeroCommand;
import frc.robot.commands.auto.DriveAndScoreConeCommand;
import frc.robot.commands.auto.DriveAndScoreCubeCommand;
import frc.robot.commands.auto.DriveDistanceAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import frc.robot.subsystems.WinchSubsystem;

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

  // private final ArmCommand m_manualPositionCommand;
  private DoubleSupplier m_dunkValue;
  private DoubleSupplier m_dunkUpValue;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  private final RuntopositionCommand m_armStoreCommand;
  private final RuntopositionCommand m_armHighCommand;
  private final RuntopositionCommand m_armMediumCommand;
  private final RuntopositionCommand m_armLowCommand;

  private final ArmCommand m_ArmMoveUpCommand = new ArmCommand(m_armSubsystem, .1);
  private final ArmCommand m_ArmMoveDownCommand = new ArmCommand(m_armSubsystem, -.05);
  private final ManualCommand m_ManualCommandUP = new ManualCommand(m_armSubsystem, true);
  private final ManualCommand m_ManualCommandDOWN = new ManualCommand(m_armSubsystem, false);
  
  private final GrabberCommandForward m_grabberCommandForward = new GrabberCommandForward(m_grabberSubsystem);
  private final GrabberCommandBackward m_grabberCommandBackward = new GrabberCommandBackward(m_grabberSubsystem);

  private final BalanceCommand m_balancecommand;
  private final DriveStraightCommand m_drivestraightcommand;
  private DriveCommand teleopDriveCmd;

  private DrivetrainSubsystem drivetrainSubsystem;

  private final ZeroCommand m_zeroCommand;

  private final DunkCommand m_dunkCommand;
  
  private final DriveAndScoreConeCommand m_autoDriveAndScoreCone;
  private final DriveAndScoreCubeCommand m_autoDriveAndScoreCube;
  private final DriveDistanceAuto m_autoDriveDistance;
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private final NavXGyroSubsystem navxGyroSubsystem = new NavXGyroSubsystem();

  private XboxController xboxController = new XboxController(ControllerConstants.kXboxControllerPort);
  private DriveStraightCommand m_autoDriveForwardCommand;
  private DriveStraightCommand m_autoDriveBackwardCommand;

  private SendableChooser<Command> m_auto_command_chooser = new SendableChooser<>();
  private WinchSubsystem m_winchSubsystem = new WinchSubsystem();

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
    return Math.abs(xboxController.getRightY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getRightY() / 5 : 0;
  }

  public double getXboxLeftY() {
    return Math.abs(xboxController.getLeftY()) > ControllerConstants.kXboxDeadZoneRadius ? -xboxController.getLeftY() : 0;
  }

  private double getThrottle() {
      return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  private double getLeftTrigger(){
    return xboxController.getLeftTriggerAxis();
  }
  
  private double getRightTrigger(){
    return xboxController.getRightTriggerAxis();
  }

  private final WinchCommand m_winchCommand = new WinchCommand(m_winchSubsystem, this::getXboxLeftY);
  private final ArmCommand m_manualPositionCommand = new ArmCommand(m_armSubsystem, this::getXboxRightY);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle, this.m_blinkinSubsystem);
    this.drivetrainSubsystem.setDefaultCommand(this.teleopDriveCmd);
    this.m_zeroCommand = new ZeroCommand(m_armSubsystem, m_winchSubsystem);
    this.m_dunkCommand = new DunkCommand(m_armSubsystem);

    this.m_balancecommand = new BalanceCommand(navxGyroSubsystem, drivetrainSubsystem);
    
    this.m_armSubsystem.setDefaultCommand(m_manualPositionCommand);
    this.m_winchSubsystem.setDefaultCommand(m_winchCommand);

    m_dunkValue = this::getRightTrigger;
    m_dunkUpValue = this::getLeftTrigger;

    this.m_drivestraightcommand = new DriveStraightCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, this::getLeftY,this::getRightY, this::getThrottle);
    this.m_armStoreCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmStoredPos, .25, m_dunkValue, m_dunkUpValue, true, m_winchSubsystem);
    this.m_armHighCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmHighPos, .25, m_dunkValue, m_dunkUpValue, true, m_winchSubsystem);
    this.m_armMediumCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmMediumPos, .25, m_dunkValue, m_dunkUpValue, true, m_winchSubsystem);
    this.m_armLowCommand = new RuntopositionCommand(m_armSubsystem, ArmConstants.kArmLowPos, .25, m_dunkValue, m_dunkUpValue, false, m_winchSubsystem);
    
    
    // this.colorSensorSubsystem.setDefaultCommand(colorSensorCommand);   <--- Causes an error right now

    // Autonomous commands
    this.m_autoDriveForwardCommand = new DriveStraightCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, ()-> 0.3 , ()-> 0.3, ()-> 1);
    this.m_autoDriveBackwardCommand = new DriveStraightCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, ()-> -0.3 , ()-> -0.3, ()-> 1);
    this.m_autoDriveAndScoreCone = new DriveAndScoreConeCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, m_armSubsystem, m_grabberSubsystem, m_winchSubsystem);
    this.m_autoDriveAndScoreCube = new DriveAndScoreCubeCommand(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, m_armSubsystem, m_grabberSubsystem, m_winchSubsystem);
    this.m_autoDriveDistance = new DriveDistanceAuto(navxGyroSubsystem, drivetrainSubsystem, m_blinkinSubsystem, 144);

    this.m_auto_command_chooser.setDefaultOption("Drive backwards for 2 seconds", m_autoDriveBackwardCommand.withTimeout(2));
    this.m_auto_command_chooser.addOption("Drive forward 2 seconds", m_autoDriveForwardCommand.withTimeout(2));
    m_auto_command_chooser.addOption("Score cube then drive backward", m_autoDriveAndScoreCube);
    m_auto_command_chooser.addOption("Score cone then drive backward", m_autoDriveAndScoreCone);
    m_auto_command_chooser.addOption("Drive X Inches", m_autoDriveDistance);
    m_auto_command_chooser.addOption("Do nothing", new PrintCommand("Do nothing"));
    Shuffleboard.getTab("Autonomous").add("Choose Autonomous Mode", m_auto_command_chooser).withSize(3, 1);
    Shuffleboard.getTab("Autonomous").add(drivetrainSubsystem);
    Shuffleboard.getTab("Autonomous").add(m_armSubsystem);
    
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
    storeButton.toggleOnTrue(m_zeroCommand);

    JoystickButton lowButton = new JoystickButton(this.xboxController, XboxController.Button.kA.value);
    lowButton.toggleOnTrue(m_armLowCommand);

    JoystickButton mediumButton = new JoystickButton(this.xboxController, XboxController.Button.kX.value);
    mediumButton.toggleOnTrue(m_armMediumCommand);
    
    JoystickButton highButton = new JoystickButton(this.xboxController, XboxController.Button.kY.value);
    highButton.toggleOnTrue(m_armHighCommand);

    JoystickButton ArmMoveUp = new JoystickButton(this.xboxController, XboxController.Button.kRightStick.value);
    ArmMoveUp.toggleOnTrue(this.m_ManualCommandUP);

    JoystickButton ArmMoveDown = new JoystickButton(this.xboxController, XboxController.Button.kLeftStick.value);
    ArmMoveDown.toggleOnTrue(this.m_ManualCommandDOWN);

    // if (xboxController.getBackButton()) this.m_zeroCommand.execute();
    JoystickButton zeroButton = new JoystickButton(this.xboxController, XboxController.Button.kBack.value);
    zeroButton.whileTrue(m_zeroCommand);

    // if (xboxController.getAButton()) this.m_dunkCommand.execute();
    JoystickButton dunkButton = new JoystickButton(this.xboxController, XboxController.Button.kStart.value);
    dunkButton.whileTrue(m_dunkCommand);

    // if (xboxController.getRightBumper()) this.m_grabberCommandForward.execute();
    JoystickButton grabForwardButton = new JoystickButton(this.xboxController, XboxController.Button.kLeftBumper.value);
    grabForwardButton.whileTrue(m_grabberCommandForward);
    // if (xboxController.getLeftBumper()) this.m_grabberCommandBackward.execute();
    JoystickButton grabBackButton = new JoystickButton(this.xboxController, XboxController.Button.kRightBumper.value);
    grabBackButton.whileTrue(m_grabberCommandBackward);

    // JoystickButton raiseWinchButton = new JoystickButton(this.leftJoystick, ControllerConstants.kRaiseWinchButton);
    // raiseWinchButton.whileTrue(m_UpWinchCommand);

    // JoystickButton lowerWinchButton = new JoystickButton(this.leftJoystick, ControllerConstants.kLowerWinchButton);
    // lowerWinchButton.whileTrue(m_DownWinchCommand);

    JoystickButton balanceButton = new JoystickButton(this.rightJoystick, ControllerConstants.kBalanceButton);
    balanceButton.whileTrue(this.m_balancecommand);

    JoystickButton driveStraightButton = new JoystickButton(this.leftJoystick, ControllerConstants.kDriveStraightButton);
    driveStraightButton.whileTrue(this.m_drivestraightcommand);
    driveStraightButton.whileFalse(this.teleopDriveCmd);
   
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auto_command_chooser.getSelected();
  }
}
