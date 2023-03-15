// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class RuntopositionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  

  private final double m_position;
  private double m_speed;
  private final double normalSpeed;
  private DoubleSupplier dunkVal;
  private DoubleSupplier dunkUpVal;
  private double m_dunkValue;
  private double m_dunkUpValue;
  private double armAdjustValue;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RuntopositionCommand(ArmSubsystem subsystem, double position, double speed, DoubleSupplier dunkValue, DoubleSupplier dunkUpValue) {
    m_armSubsystem = subsystem;
    m_position = position;
    normalSpeed = speed;
    m_speed = speed;
    System.out.println(dunkValue);
    dunkVal = dunkValue;
    dunkUpVal = dunkUpValue;
    
    
   
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
    System.out.println((m_position - 6 * armAdjustValue));
    ArmSubsystem.runToPosition(ArmSubsystem.m_extender, m_armSubsystem.m_extenderEncoder, (m_position - (6 * armAdjustValue)), m_speed);
    //System.out.println(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.runExtender(0);
    m_armSubsystem.stopMotor(ArmSubsystem.m_extender);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
