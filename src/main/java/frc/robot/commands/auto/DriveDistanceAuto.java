// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

/** An example command that uses an example subsystem. */
public class DriveDistanceAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDistanceAuto(NavXGyroSubsystem gyro, DrivetrainSubsystem drive, BlinkinSubsystem blinkin, double distance) {
    super(
      // TODO: Make this drive distance and/or line up with the cone before the RunToPosition command should run

      new DriveDistanceCommand(gyro, drive, blinkin, .4, .4, 1, distance)
    );
  }

}
