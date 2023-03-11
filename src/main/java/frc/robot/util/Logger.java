package frc.robot.util;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.StringJoiner;
import java.util.function.Supplier;

/**
 * Responsible for recording the sensor data
 */
public class Logger {

    private String path;
    private File file;

    List<DynamicField> dynamicFields;

    // TODO: Look into the datalog tool to be able to automatically pull the log file off the robot
    // TODO: Use the RobotSimulation to test the logging utility
    // TODO: Track Autonomous and Teleop Status
    public Logger(String p, String name) {
        path = p;
        try {
            file = new File(path + name + ".csv");
            if (file.createNewFile()) {
                System.out.println("File Created: " + file.getName());
            } else {
                System.out.println("File already exists.");
            }

            // Wipes the log if it already contains text
            FileWriter myWriter = new FileWriter(file,false);
            myWriter.close();

        } catch (IOException e) {
            System.out.println("Unable to create a file. ");
            e.printStackTrace();
        }

        dynamicFields = new ArrayList<>();
    }

    public void log() {
        try {
            // TODO: Do not close the writer after each write
            FileWriter myWriter = new FileWriter(file,true);

            myWriter.write("\n");

            List<String> values = new ArrayList<>();

            for(DynamicField dF : dynamicFields) {
                values.add(dF.getValue().toString());
            }

            myWriter.write(String.join(",", values));
            myWriter.close();
        } catch (IOException e) {
            System.out.println("Could not write to file.");
            e.printStackTrace();
        }
    }

    /**
     * Sets up the Dynamic Fields 
     */ 
    public void setup() {
        try {
            FileWriter myWriter = new FileWriter(file,true);

            //myWriter.write("Created using FREZ Log");

            myWriter.write("---\n");

            List<String> variableNames = new ArrayList<>();

            for(DynamicField dF : dynamicFields) {
                variableNames.add(dF.getName());
            }

            myWriter.write(String.join(",", variableNames));

            myWriter.close();
        } catch (IOException e) {
            System.out.println("Could not write to file.");
            e.printStackTrace();
        }
    }

    public void createDynamicFieldDouble(String n, Double v, Supplier<Double> s){
        // This causes a runtime error which can prevent logging a value of a wrong type
        createDynamicField(n, v, () -> s.get());
    }

    public void createDynamicFieldString(String n, Object v, Supplier<String> s){
        // This causes a runtime error which can prevent logging a value of a wrong type
        createDynamicField(n, v, () -> s.get());
    }

    public void createDynamicFieldBoolean(String n, Boolean v, Supplier<Boolean> s){
        // This causes a runtime error which can prevent logging a value of a wrong type
        createDynamicField(n, booleanToInt(v), () -> booleanToInt(s.get()));
    }

    private void createDynamicField(String n, Object v, Supplier<Object> s) {
        DynamicField newField = new DynamicField(n, v, s);
        dynamicFields.add(newField);
    }

    public void createStaticField(String n, Object v) {
        StaticField newField = new StaticField(n, v);
        writeStaticField(n, v);
    }

    /**
     * Records the Frequency of the Logs (does not determine how often they are actually created)
     *
     * @param timeUnit - decimal of a second that represents how often the logs are updated
     */
    public void recordFrequency(double timeUnit) {
        // t? is the property tag to signify time unit
        createStaticField("t?Time Unit", timeUnit);
    }

    public void writeStaticField(String n, Object v) {
        try {
            FileWriter myWriter = new FileWriter(file,true);

            myWriter.write(n+","+v+"\n");

            myWriter.close();
        } catch (IOException e) {
            System.out.println("Could not write to file.");
            e.printStackTrace();
        }
    }

    public void updateFields() {
        for(DynamicField field : dynamicFields) {
            field.update();
        }

        System.out.println("Updated Dynamic Fields.");
    }

    public int booleanToInt(boolean bool) {
        if(bool)
            return 1;
        return 0;
    }
}
