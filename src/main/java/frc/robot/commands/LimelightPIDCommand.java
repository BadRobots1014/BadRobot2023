package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightPIDCommand extends PIDCommand{

    public LimelightPIDCommand(LimelightSubsystem lS, DrivetrainSubsystem dS, double power) {
        super(new PIDController(LimelightConstants.kP, LimelightConstants.kI, LimelightConstants.kD),
            lS::getTableX, 
            LimelightConstants.setpoint, 
            output -> dS.tankDrive(power * output, power * -output),
           // output -> dS.tankDrive(power * output, power * (1-output)), 
            new Subsystem[]{lS, dS});
    }
    
}
