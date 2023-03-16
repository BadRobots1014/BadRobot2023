// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.DunkCommand;
import frc.robot.commands.GrabberCommandBackward;
import frc.robot.commands.RuntopositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

/** An example command that uses an example subsystem. */
public class DriveAndScoreCubeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveAndScoreCubeCommand(NavXGyroSubsystem gyro, DrivetrainSubsystem drive, BlinkinSubsystem blinkin, ArmSubsystem arm, GrabberSubsystem grabber) {
    super(
      // TODO: Make this drive distance and/or line up with the cone before the RunToPosition command should run
      new RuntopositionCommand(arm, ArmConstants.kArmMediumPos, .25).withTimeout(2),
      new DunkCommand(arm).withTimeout(1),
      new ParallelRaceGroup(new DunkCommand(arm), new GrabberCommandBackward(grabber)).withTimeout(.5),
      new ParallelRaceGroup(new DriveStraightCommand(gyro, drive, blinkin, .5, .5, 1), new RuntopositionCommand(arm, ArmConstants.kArmStoredPos, .2))
    );
  }

}
