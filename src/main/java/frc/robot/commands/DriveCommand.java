// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_subsystem;
  private final BlinkinSubsystem m_ledSubsystem;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_zRotation;
  private DoubleSupplier m_throttle;
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
  // private GenericEntry directionEntry =
  //      m_tab.add("Direction", "")
  //         .getEntry();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zRotation, DoubleSupplier throttle, BlinkinSubsystem lightSubsystem) {
    m_subsystem = subsystem;
    m_ledSubsystem = lightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(lightSubsystem);

    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRotation = zRotation;
    m_throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(m_xSpeed.getAsDouble() * m_throttle.getAsDouble(), m_ySpeed.getAsDouble() * m_throttle.getAsDouble(), m_zRotation.getAsDouble() * m_throttle.getAsDouble());

    // // String lightDirection = m_subsystem.getDirection(m_leftSpeed.getAsDouble() * m_throttle.getAsDouble(), m_rightSpeed.getAsDouble() * m_throttle.getAsDouble());
    // directionEntry.setString(lightDirection);
    // switch(lightDirection) {
    //   case(MovementConstants.kStationary):
    //     m_ledSubsystem.set(BlinkinPatternConstants.kSolidWhite);
    //     break;
    //   case(MovementConstants.kForward):
    //     if(m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle){
    //       m_ledSubsystem.set(BlinkinPatternConstants.kStrobeBlue);
    //     } else {
    //       m_ledSubsystem.set(BlinkinPatternConstants.kSolidBlue);
    //     }
    //     break;
    //   case(MovementConstants.kBackward):
    //     if(m_throttle.getAsDouble() == ControllerConstants.kSlowThrottle){
    //       m_ledSubsystem.set(BlinkinPatternConstants.kStrobeRed);
    //     } else {
    //       m_ledSubsystem.set(BlinkinPatternConstants.kSolidRed);
    //     }
    //     break;
    //   default:
    //     m_ledSubsystem.set(BlinkinPatternConstants.kSolidWhite);
    //     break;
    // }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
