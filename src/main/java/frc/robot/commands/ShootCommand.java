package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

    /*
     * Private constants ------------------------------------------------------
     */

    /**
     * The amount of time, in seconds, to wait between activating the shooter motor
     * and activating the indexer motor.
     */
    public static final double WAIT_TIME = 2.0;

    /*
     * Private instance members -----------------------------------------------
     */

    /**
     * The {@link ShooterSubsystem} controlled by this command.
     */
    private final ShooterSubsystem shooter;

    /**
     * The {@link IndexerSubsystem} controlled by this command.
     */
    private final IndexerSubsystem indexer;

    /**
     * A {@link Timer} used to implement a delay between activating the shooter
     * motor and activating the indexer motor.
     */
    private final Timer timer;

    /**
     * The percentage output to drive the shooter at.
     */
    private final double power;

    /*
     * Shuffleboard -----------------------------------------------------------
     */

    private final String name;

    private final ShuffleboardTab shuffleTab;
    private final NetworkTableEntry powerEntry;

    /*
     * Constructor ------------------------------------------------------------
     */

    /**
     * Constructs a new {@link ShootCommand}, parameterized by the subsystems
     * ({@code shooter}, {@code indexer}) to control, and the shooter power output.
     * 
     * @param shooter the {@link ShooterSubsystem} controlled by this command
     * @param indexer the {@link IndexerSubsystem} controlled by this command
     * @param power   the percentage output to run the shooter at
     * @param name    the name to display for this command in Shuffleboard
     * @requires {@code power} is in the range [-1.0, 1.0]
     */
    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, double power, String name) {
        this.shooter = shooter;
        this.indexer = indexer;

        this.timer = new Timer();

        this.power = power;

        this.name = name;

        this.shuffleTab = Shuffleboard.getTab("Shooter");
        this.powerEntry = this.shuffleTab.add("Power: " + this.name, this.power).withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        addRequirements(shooter, indexer);
    }

    /*
     * Command methods --------------------------------------------------------
     */

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();

        this.shooter.run(this.powerEntry.getDouble(this.power));
    }

    @Override
    public void execute() {
        this.shooter.run(this.powerEntry.getDouble(this.power));

        if (this.timer.hasElapsed(WAIT_TIME)) {
            this.indexer.runUpperMotor(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        this.indexer.stopUpperMotor();
        System.out.println("Stopping everything");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
