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
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipperCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.commands.blinkin.ShootBlinkinCommand;
import frc.robot.commands.blinkin.SpinUpBlinkinCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private DriveCommand teleopDriveCmd;
  private DrivetrainSubsystem drivetrainSubsystem;
  
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  private ShooterSubsystem shooterSubsystem;
  private SpinUpCommand shootCommand;

  private FlipperCommand flipperCommand;
  private FlipperCommand flipperBackCommand;
  private FlipperSubsystem flipperSubsystem;

  private BlinkinSubsystem blinkinSubsystem;

  private SpinUpBlinkinCommand spinUpBlinkinCommand;
  private ShootBlinkinCommand shootBlinkinCommand;

  public double getRightY() {
    return -rightJoystick.getY();
  }

  public double getLeftY() {
    return -leftJoystick.getY();
  }

  public double getRightZ() {
    return -((rightJoystick.getZ() - 1) / 2);
  }

  private double getThrottle() {
    return this.rightJoystick.getRawButton(ControllerConstants.kThrottleButton) ? ControllerConstants.kSlowThrottle : ControllerConstants.kMaxThrottle;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    this.drivetrainSubsystem = new DrivetrainSubsystem();
    this.shooterSubsystem = new ShooterSubsystem();
    this.flipperSubsystem = new FlipperSubsystem();
    this.flipperSubsystem.setDefaultCommand(flipperBackCommand);
    this.blinkinSubsystem = new BlinkinSubsystem();

    this.rightJoystick = new Joystick(ControllerConstants.kRightJoystickPort);
    this.leftJoystick = new Joystick(ControllerConstants.kLeftJoystickPort);
    
    this.teleopDriveCmd = new DriveCommand(this.drivetrainSubsystem, this::getRightY, this::getLeftY, this::getThrottle);
    this.shootCommand = new SpinUpCommand(this.shooterSubsystem, this::getRightZ);
    this.flipperCommand = new FlipperCommand(this.flipperSubsystem, ShooterConstants.kFlipperPower);
    this.flipperBackCommand = new FlipperCommand(this.flipperSubsystem, ShooterConstants.kFlipperBackPower);
    this.spinUpBlinkinCommand = new SpinUpBlinkinCommand(this.blinkinSubsystem);
    this.shootBlinkinCommand = new ShootBlinkinCommand(this.blinkinSubsystem);

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
    
    JoystickButton spinUpButton = new JoystickButton(this.rightJoystick, ControllerConstants.kSpinUpButton);
    spinUpButton.whileTrue(this.shootCommand);
    spinUpButton.whileTrue(this.spinUpBlinkinCommand);

    double runTime = 2;
    JoystickButton shootButton = new JoystickButton(this.leftJoystick, ControllerConstants.kShootButton);
    shootButton.onTrue(this.flipperCommand.withTimeout(runTime).andThen(this.flipperBackCommand.withTimeout(runTime)));
    shootButton.onTrue(this.shootBlinkinCommand.withTimeout(runTime));

    JoystickButton pullBackButton = new JoystickButton(this.leftJoystick, ControllerConstants.kShootBackButton);
    pullBackButton.whileTrue(this.flipperBackCommand);

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
