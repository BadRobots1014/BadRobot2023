// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.SwerveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_rightJoystick = new Joystick(0);
  Joystick m_leftJoystick = new Joystick(1);

  //Paths
  private PathPlannerTrajectory m_autoTraj;
  private PathPlannerPath m_autoPath;
  private PathPlannerAuto m_auto;

  //TEST
  private double m_testMotorId = 0;
  private double m_testMotorSpeed = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // m_robotDrive.setDefaultCommand(new SwerveDriveCommand(
    //   m_robotDrive,
    //   () -> m_driverController.getLeftX(),
    //   () -> m_driverController.getLeftY(),
    //   () -> m_driverController.getRightX(),
    //   () -> DriveConstants.kFieldOriented
    // ));
    m_robotDrive.setDefaultCommand(new SwerveDriveCommand(
      m_robotDrive,
      m_supplyDouble(1),
      m_supplyDouble(0),
      m_supplyDouble(0),
      m_supplyBool(true)
    ));

    // Configure the button bindings
    configureButtonBindings();

    //Setup paths
    m_autoPath = PathPlannerPath.fromPathFile("New Path");
    // m_autoTraj = new PathPlannerTrajectory(m_autoPath, m_robotDrive.getModuleStates(), m_robotDrive.getRotation2d());
  }

  private Supplier<Double> m_supplyDouble(double num) {
    return new Supplier<Double>() {
      @Override
      public Double get() {
          // TODO Auto-generated method stub
          return num;
      }
    };
  }

  private Supplier<Boolean> m_supplyBool(boolean bool) {
    return new Supplier<Boolean>() {
      @Override
      public Boolean get() {
          // TODO Auto-generated method stub
          return bool;
      }
    };
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kStart.value).whileTrue(new ZeroHeadingCommand(m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_auto = new PathPlannerAuto("New Auto");
    return m_auto;
    // return m_robotDrive.followTrajectoryCommand(m_autoTraj, m_autoPath, true);
  }
}