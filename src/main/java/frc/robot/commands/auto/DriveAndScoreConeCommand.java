// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.DunkCommand;
import frc.robot.commands.GrabberCommandForward;
import frc.robot.commands.RuntopositionCommand;
import frc.robot.commands.ZeroCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

/** An example command that uses an example subsystem. */
public class DriveAndScoreConeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveAndScoreConeCommand(NavXGyroSubsystem gyro, DrivetrainSubsystem drive, BlinkinSubsystem blinkin, ArmSubsystem arm, GrabberSubsystem grabber, ArmSubsystem winch) {
    super(
      // TODO: Make this drive distance and/or line up with the cone before the RunToPosition command should run

      new ZeroCommand(arm).withTimeout(2),
      new RuntopositionCommand(arm, ArmConstants.kArmHighPos, .25, null, null, true, winch).withTimeout(2),
      new ParallelCommandGroup(new DriveStraightCommand(gyro, drive, blinkin, .3, .3, 1),  new RuntopositionCommand(arm, ArmConstants.kArmHighPos, .25, null, null, true, winch)).withTimeout(3),
      new ParallelCommandGroup(new GrabberCommandForward(grabber).withTimeout(1.5), new RuntopositionCommand(arm, ArmConstants.kArmHighPos, .25, null, null, true, winch)).withTimeout(2),
      new ParallelCommandGroup(new DriveStraightCommand(gyro, drive, blinkin, -.45, -.45, 1), new RuntopositionCommand(arm, ArmConstants.kArmStoredPos, .4, null, null, true, winch)).withTimeout(3)
    );
  }

}
