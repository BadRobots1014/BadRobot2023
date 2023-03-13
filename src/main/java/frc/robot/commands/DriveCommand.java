
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.BlinkinPatternConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MovementConstants;
import frc.robot.subsystems.BlinkinSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem m_subsystem;
  private final BlinkinSubsystem m_ledSubsystem;
  private DoubleSupplier m_rightSpeed;
  private DoubleSupplier m_leftSpeed;
  private DoubleSupplier m_throttle;
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
  private GenericEntry directionEntry = m_tab.add("Direction", "").getEntry();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier rightSpeed, DoubleSupplier leftSpeed, DoubleSupplier throttle, BlinkinSubsystem lightSubsystem) {
    m_subsystem = subsystem;
    m_ledSubsystem = lightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(lightSubsystem);

    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    m_throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.tankDrive(m_leftSpeed.getAsDouble() * m_throttle.getAsDouble(),
        m_rightSpeed.getAsDouble() * m_throttle.getAsDouble());
    // Grabs light direction signifier
    String lightDirection = m_subsystem.getDirection(m_leftSpeed.getAsDouble() * m_throttle.getAsDouble(),
        m_rightSpeed.getAsDouble() * m_throttle.getAsDouble());
    directionEntry.setString(lightDirection);
    

    // Switch Case, sets colors based on the lightdirection signifier
    switch (lightDirection) {
     
      // White if Stationary
      // Light Test Mode, Default Should be white, is currently commented, if not testing switch commented status
      case (MovementConstants.kStationary):
        m_ledSubsystem.set(BlinkinPatternConstants.solidWhite);
        break;
     
      case(MovementConstants.kForward):
        if(m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle){
          m_ledSubsystem.set(BlinkinPatternConstants.breatheBlue);

        } else {
          m_ledSubsystem.set(BlinkinPatternConstants.solidBlue);
        }
        break;

      // Red if Backwards
      case(MovementConstants.kBackward):
        if(m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle){
          m_ledSubsystem.set(BlinkinPatternConstants.breatheRed);

        } else {
          m_ledSubsystem.set(BlinkinPatternConstants.solidRed);
        }
        break;

      // Color 1 If Turning Counterclockwise
      case (MovementConstants.kTurningCounterclockwise):
        if (m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle) {
          m_ledSubsystem.set(BlinkinPatternConstants.breatheColor1);
        } else {
          m_ledSubsystem.set(BlinkinPatternConstants.solidGreen);
        }
        break;

      // Color 2 If Turning Clockwise.
      case (MovementConstants.kTurningClockwise):
        if (m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle) {
          m_ledSubsystem.set(BlinkinPatternConstants.breatheColor2);
        } else {
          m_ledSubsystem.set(BlinkinPatternConstants.solidOrange);
        }
        break;

      // Confetti When Spinning in Place
      case (MovementConstants.kSpinningInPlace):
        m_ledSubsystem.set(BlinkinPatternConstants.confetti);
        break;
      default:
        m_ledSubsystem.set(BlinkinPatternConstants.solidWhite);
        break;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
