package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

public class BalancePIDCommand extends PIDCommand{
    private static boolean isFirst = true;


    public BalancePIDCommand(NavXGyroSubsystem gyroSub, DrivetrainSubsystem driveSub) {
        super(new PIDController(GyroConstants.kGyroP, GyroConstants.kGyroI, GyroConstants.kGyroD),
        gyroSub::getRollPID,
        GyroConstants.kGyroSetpoint,
        output -> driveAndLog(gyroSub, driveSub, output),
        new Subsystem[]{gyroSub, driveSub});
    }

    private static void driveAndLog(NavXGyroSubsystem gyroSub, DrivetrainSubsystem driveSub, double output) {
        //m_tab.addNumber("PID Output", output);
        System.out.println("Gyro Roll: " + gyroSub.getRollPID() + "\tControl Effort: " + output + "\tLMotor: " + (GyroConstants.kMaxBalanceSpeed * output) + "\tRMotor: " + (GyroConstants.kMaxBalanceSpeed * output));
        if(isFirst) {
            isFirst = false;
            return;
        }
        driveSub.tankDrive(GyroConstants.kMaxBalanceSpeed * output, GyroConstants.kMaxBalanceSpeed * output);
        //dS.tankDrive(.5, .5);
        
    }
}
