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

public class LimelightAllignPIDCommand extends PIDCommand{
    public final static ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    private static boolean isFirst = true;
    private static double[] lastVals;

    private static LimelightSubsystem m_LimelightSubsystem;

    public LimelightAllignPIDCommand(LimelightSubsystem lS, DrivetrainSubsystem dS) {
        super(new PIDController(LimelightConstants.kAngleP, LimelightConstants.kAngleI, LimelightConstants.kAngleD),
        lS::getPIDTurnTableX, 
        LimelightConstants.angleSetpoint, 
        //output -> dS.tankDrive(power * output, power * -output),
        //output -> System.out.println(output),
        //output -> dS.tankDrive(power * output, power * (1-output)), 
        output -> driveAndLog(lS,dS, output),
        new Subsystem[]{lS, dS});

        m_LimelightSubsystem = lS;
    }
    
    private static void driveAndLog(LimelightSubsystem lS, DrivetrainSubsystem dS, double output) {
        //m_tab.addNumber("PID Output", output);
        System.out.println("LL tX: " + lS.getPIDTurnTableX() + 
                            "\tControl Effort: " + output + 
                            "\tLMotor: " + (LimelightConstants.kLineUpMaxSpeed * -output) + 
                            "\tRMotor: " + (LimelightConstants.kLineUpMaxSpeed * output));
        if(isFirst) {
            isFirst = false;
            return;
        }
        dS.tankDrive(LimelightConstants.kLineUpMaxSpeed * -output, LimelightConstants.kLineUpMaxSpeed * output);
        //dS.tankDrive(.5, .5);
        
    }

    @Override
    public void initialize() {
        super.initialize();
        isFirst = true;

        m_LimelightSubsystem.setPipeline(2);
    }

}
