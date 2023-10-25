// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WinchSubsystem;

/** An example command that uses an example subsystem. */
public class RuntopositionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final WinchSubsystem m_winchSubsystem;

  private final double m_position;
  private double m_speed;
  private final double normalSpeed;
  private DoubleSupplier dunkVal;
  private DoubleSupplier dunkUpVal;
  private double m_dunkValue;
  private double m_dunkUpValue;
  private double armAdjustValue;
  private boolean m_winchUp;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public RuntopositionCommand(ArmSubsystem subsystem, double position, double speed, DoubleSupplier dunkValue, DoubleSupplier dunkUpValue, boolean winchUp, WinchSubsystem winchSubsystem) {

    m_armSubsystem = subsystem;
    m_winchSubsystem = winchSubsystem;
    m_position = position;
    normalSpeed = speed;
    m_speed = speed;

    System.out.println(dunkValue);
    dunkVal = dunkValue;
    dunkUpVal = dunkUpValue;
    m_winchUp = winchUp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(dunkVal == null){
      m_dunkValue = 0;
    }
    else{
      m_dunkValue = dunkVal.getAsDouble();
      //System.out.println(m_dunkValue);
    }
    if(dunkUpVal == null){
      m_dunkUpValue = 0;
    }
    else{
      m_dunkUpValue = dunkUpVal.getAsDouble();
      //System.out.println(m_dunkValue);
    }

    if(Math.abs(m_dunkValue) <= 0.05){
      m_dunkValue = 0;
    }
    if(Math.abs(m_dunkUpValue) <= 0.05){
      m_dunkUpValue = 0;
    }

   // m_dunkUpValue = 1;
   //System.out.println(m_dunkValue);
    armAdjustValue = m_dunkValue - m_dunkUpValue;

    

    if(m_armSubsystem.getExtenderEncoderPosition() <= 0){
      armAdjustValue = 0;
    }
    if(m_armSubsystem.getExtenderEncoderPosition() >= 40){
      armAdjustValue = 0;
    }



    if(m_armSubsystem.getExtenderEncoderPosition() != m_position - (6 * armAdjustValue) && (m_dunkUpValue != 0 || m_dunkValue != 0)){
      m_speed = 0.05;
    }else{
      m_speed = normalSpeed;
    }
    //System.out.println(armAdjustValue);

    //System.out.println(m_dunkValue);
    //System.out.println(m_position - (2 * m_dunkValue));
    System.out.println((m_position - 8 * armAdjustValue));
    m_armSubsystem.runToPosition(m_armSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, (m_position - (8 * armAdjustValue)), m_speed);

    //Note that this only runs if the extender is within X units of the correct position to prevent collisions
    // m_armSubsystem.runWinch(m_winchUp ? ((Math.abs(m_armSubsystem.getEncoderPosition(m_armSubsystem.m_extenderEncoder) - m_position) < ArmConstants.kWinchLowerDeadzone) ? ArmConstants.kWinchUpSpeed : 0) : ArmConstants.kWinchDownSpeed);
    // m_armSubsystem.runWinch(ArmConstants.kWinchUpSpeed);

    if (Math.abs(m_armSubsystem.getEncoderPosition(m_armSubsystem.m_extenderEncoder) - m_position) < ArmConstants.kWinchLowerDeadzone) System.out.print((m_position - (6 * armAdjustValue)));
    //System.out.println(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.runExtender(0);
    m_armSubsystem.stopMotor(m_armSubsystem.m_extender);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
