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

public class LimelightDistancePIDCommand extends PIDCommand{
    
    public final static ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    private static boolean isFirst = true;

    private static LimelightSubsystem m_LimelightSubsystem;

    // TODO: remove power because it doesn't get used
    public LimelightDistancePIDCommand (LimelightSubsystem lS, DrivetrainSubsystem dS, double power, 
                                        double llMADeg, double llMH, double tH) {

        super(new PIDController(LimelightConstants.kDistP, LimelightConstants.kDistI, LimelightConstants.kDistD),
        () -> lS.calculateDistanceInch(llMADeg, llMH, tH), 
        LimelightConstants.distSetpoint, 
        //output -> dS.tankDrive(power * output, power * -output),
        //output -> System.out.println(output),
        //output -> dS.tankDrive(power * output, power * (1-output)), 
        output -> driveAndLog(lS,dS,power, output, llMADeg, llMH, tH),
        new Subsystem[]{lS, dS});      
        
        m_LimelightSubsystem = lS;
    }
    
    private static void driveAndLog(LimelightSubsystem lS, DrivetrainSubsystem dS, double power, double output,
                                    double llMADeg, double llMH, double tH) {
        //m_tab.addNumber("PID Output", output);
        System.out.println("LL tX: " + lS.calculateDistanceInch(llMADeg, llMH, tH) + "\tControl Effort: " + output + "\tLMotor: " + (power * output) + "\tRMotor: " + (power * output));
        if(isFirst) {
            isFirst = false;
            return;
        }

        if(m_LimelightSubsystem.getTableY() != 0.0) // no target detected
            dS.tankDrive(LimelightConstants.kDistMaxSpeed * output, LimelightConstants.kDistMaxSpeed * output);
        //dS.tankDrive(.5, .5);
    }


    @Override
    public void initialize() {
        super.initialize();
        isFirst = true;

        m_LimelightSubsystem.setPipeline(1);
    }
}
