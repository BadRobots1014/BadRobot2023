package frc.robot.util;

import java.util.function.Supplier;

public class DynamicField extends StaticField {

    private Supplier<Object> supplier;

    /**
     * A supplier "broker". Stores the name of the measurement, latest value, and the supplier of the values.
     * Uses Object as a flexible data type.
     *
     * @param n     field name
     * @param v     initial value
     * @param s     value supplier/source
     */
    public DynamicField(String n, Object v, Supplier<Object> s) {
        super(n,v);

        supplier = s;
    }

    public void update() {
        super.setValue(supplier.get());
    }
}
