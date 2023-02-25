package frc.robot.util;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * Struct for storing the sensor data field
 */

public class StaticField{
    private String name;
    private Object value;

    public StaticField(String n, Object v) {
        name = n;
        value = v;
    }

    public Object getValue() {
        return value;
    }

    public void setValue(Object v) {
        value = v;
    }

    public String getName() {
        return name;
    }
}
