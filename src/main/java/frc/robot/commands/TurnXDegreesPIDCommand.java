package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.GyroConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

public class TurnXDegreesPIDCommand extends PIDCommand{
    public final static ShuffleboardTab m_tab = Shuffleboard.getTab("Drivetrain");
    private static boolean isFirst = true;
    private double setpoint;

    public TurnXDegreesPIDCommand(NavXGyroSubsystem gS, DrivetrainSubsystem dS, double setpoint) {
        super(new PIDController(GyroConstants.kP, GyroConstants.kI, GyroConstants.kD),
        gS::getPIDYaw, 
        gS.getPIDYaw() + setpoint/180, 
        //output -> dS.tankDrive(power * output, power * -output),
        //output -> System.out.println(output),
        //output -> dS.tankDrive(power * output, power * (1-output)), 
        output -> driveAndLog(gS,dS, output),
        new Subsystem[]{gS, dS});

        this.setpoint = setpoint;
    }
    
    private static void driveAndLog(NavXGyroSubsystem gS, DrivetrainSubsystem dS, double output) {
        //m_tab.addNumber("PID Output", output);
        System.out.println("Yaw: " + gS.getYaw() + "\tControl Effort: " + output + "\tLMotor: " + (GyroConstants.kLineUpMaxSpeed * -output) + "\tRMotor: " + (GyroConstants.kLineUpMaxSpeed * output));
        if(isFirst) {
            isFirst = false;
            return;
        }
        dS.tankDrive(GyroConstants.kLineUpMaxSpeed * -output, GyroConstants.kLineUpMaxSpeed * output);
        //dS.tankDrive(.5, .5);
        
    }

    @Override
    public void initialize() {
        super.initialize();

        super.getController().setSetpoint(setpoint);

        isFirst = true;
    }
}