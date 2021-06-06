package org.ma.ftc.lib;

public class MathUtil {

    public static double clamp(double value, double max, double min) {
        return value > max ? max : (Math.max(value, min));
    }

    public static void normalize(double[] values) {
        double maxValue = Math.abs(values[0]);
        for (int i = 1; i < values.length; i++) {
            double speed = Math.abs(values[i]);
            if (speed > maxValue) {
                maxValue = speed;
            }
        }
        if (maxValue > 1.0) {
            for (int i = 0; i < values.length; i++) {
                values[i] /= maxValue;
            }
        }
    }
}