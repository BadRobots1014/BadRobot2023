package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonSRX m_lowerMotor = new TalonSRX(IndexerConstants.kLowerIndexerSpeedController);
    private final TalonSRX m_upperMotor = new TalonSRX(IndexerConstants.kUpperIndexerSpeedController);

    private final DigitalInput m_lowerSensor = new DigitalInput(IndexerConstants.kLowerIndexerSensor);
    private final DigitalInput m_upperSensor = new DigitalInput(IndexerConstants.KUpperIndexerSensor);
    private final BooleanSupplier m_lowerSensorSupplier = () -> m_lowerSensor.get();
    private final BooleanSupplier m_upperSensorSupplier = () -> m_upperSensor.get();

    private String m_state;
    private Timer m_timer;

    private final ShuffleboardTab tab;

    public IndexerSubsystem() {
        overideState("empty");
        m_timer = new Timer();

        this.tab = Shuffleboard.getTab("Indexer");
        this.tab.addBoolean("Lower Sensor", m_lowerSensorSupplier);
        this.tab.addBoolean("Upper Sensor", m_upperSensorSupplier);
        this.tab.addString("State", () -> m_state);
        this.tab.addBoolean("Time Elapsed", () -> m_timer.hasElapsed(5));
    }

    public void periodic() {
        checkState();
        actOnState();
    }

    public void runLowerMotor(double power) {
        m_lowerMotor.set(ControlMode.PercentOutput, IndexerConstants.kIndexerMaxSpeed * power);
        //System.out.println("Running lower motor");
    }

    public void stopLowerMotor() {
        m_lowerMotor.set(ControlMode.PercentOutput, 0);
        //System.out.println("Stopped lower motor");
    }

    public void runUpperMotor(double power) {
        m_upperMotor.set(ControlMode.PercentOutput, -IndexerConstants.kIndexerMaxSpeed * power);
        //System.out.println("Running upper motor");
    }

    public void stopUpperMotor() {
        m_upperMotor.set(ControlMode.PercentOutput, 0);
        //System.out.println("Stopped upper motor");
    }

    public void stopAll() {
        m_upperMotor.set(ControlMode.PercentOutput, 0);
        m_lowerMotor.set(ControlMode.PercentOutput, 0);
        //System.out.println("Stopped indexer motors");
    }

    public boolean lowerSensorOn() {
        return m_lowerSensor.get();
    }

    public boolean upperSensorOn() {
        return m_upperSensor.get();
    }

    public void checkState() { //Moves through the natural state flow
        switch(m_state) {
            case "empty":
                if (lowerSensorOn()) {
                    m_state = "emptyIntaking";
                }
            break;
            case "emptyIntaking":
                m_timer.reset();
                m_timer.start();
                if (upperSensorOn()) {
                    m_state = "emptyIntaking2"; //TODO Run the rest of the way up
                    m_timer.reset();
                    m_timer.start();
                }
                else if (m_timer.hasElapsed(5)) {
                    m_timer.stop();
                    jumpState();
                }
                //TODO Stop if the ball rolls out? (Not for now)
            break;
            case "emptyIntaking2":
                if (m_timer.hasElapsed(IndexerConstants.kIndexTime/2)) {
                    m_timer.stop();
                    m_state = "topLoaded";
                }
            break;
            case "fullLoadedDelay":
                if (m_timer.hasElapsed(IndexerConstants.kIndexTime*2)) {
                    m_timer.stop();
                    m_state = "emptyIntaking"; //TODO Run the rest of the way up
                }
            break;
            case "topLoaded":
                if (!upperSensorOn()) {
                    m_state = "empty";
                }
                else if (lowerSensorOn()) {
                    m_state = "topLoadedIntaking";
                    m_timer.reset();
                    m_timer.start();
                }
            break;
            case "topLoadedIntaking":
                if (!lowerSensorOn()) {
                    m_timer.stop();
                    m_state = "topLoaded";
                }
                else if (m_timer.hasElapsed(IndexerConstants.kIndexTime)) {
                    m_timer.stop();
                    m_state = "fullLoaded";
                }
            break;
            case "fullLoaded":
                if (!upperSensorOn()) {
                    m_state = "fullLoadedDelay";
                    m_timer.reset();
                    m_timer.start();
                }
                else if (!lowerSensorOn()) {
                    m_state = "topLoaded";
                }
            break;
        }
    }

    public void jumpState() { //Gets the state by raw sensor data, not state flow
        if (upperSensorOn()) {
            if (lowerSensorOn()) m_state = "fullLoaded";
            else m_state = "topLoaded";
        }
        else {
            if (lowerSensorOn()) m_state = "emptyIntaking";
            else m_state = "empty";
        }
        //Does not include "topLoadedIntaking" state.
    }

    public void overideState(String newState) { //Sets the state
        m_state = newState;
    }

    public void actOnState() {
        switch(m_state) {
            case "empty":
            case "topLoaded":
            case "fullLoaded":
            case "fullLoadedDelay":
                stopLowerMotor();
            break;
            case "emptyIntaking":
            case "emptyIntaking2":
            case "topLoadedIntaking":
                runLowerMotor(1);
            break;
        }
    }
}
