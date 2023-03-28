package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightPIDCommand extends PIDCommand{
    public final static ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    private static boolean isFirst = true;
    private static double[] lastVals;

    public LimelightPIDCommand(LimelightSubsystem lS, DrivetrainSubsystem dS, double power) {
        super(new PIDController(LimelightConstants.kP, LimelightConstants.kI, LimelightConstants.kD),
        lS::getTableX, 
        LimelightConstants.setpoint, 
        output -> driveAndLog(lS,dS,power, output),
        new Subsystem[]{lS, dS});
    }
    
    private static void driveAndLog(LimelightSubsystem lS, DrivetrainSubsystem dS, double power, double output) {
        //m_tab.addNumber("PID Output", output);
        System.out.println("LL tX: " + lS.getTableX() + "\tControl Effort: " + output + "\tLMotor: " + (LimelightConstants.kLineUpMaxSpeed * -output) + "\tRMotor: " + (LimelightConstants.kLineUpMaxSpeed * output));
        if(isFirst) {
            isFirst = false;
            return;
        }
        //dS.tankDrive(LimelightConstants.kLineUpMaxSpeed * -output, LimelightConstants.kLineUpMaxSpeed * output);
        //dS.tankDrive(.5, .5);
        
    }

    @Override
    public void initialize() {
        super.initialize();
        isFirst = true;
    }
}
