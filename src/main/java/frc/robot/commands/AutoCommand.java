// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

/** An example command that uses an example subsystem. */
public class AutoCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveStraightCommand m_drive;
  private RuntopositionCommand m_arm;
  private Timer timer;
  private boolean finish = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoCommand(NavXGyroSubsystem gyro, DrivetrainSubsystem drive, BlinkinSubsystem blinkin, ArmSubsystem arm) {
    super(
      new DriveStraightCommand(gyro, drive, blinkin, .2, .2, 1).withTimeout(2000),
      new RuntopositionCommand(arm, 20, .1)
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

}
