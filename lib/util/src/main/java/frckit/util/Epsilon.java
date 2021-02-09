package frckit.util;

public class Epsilon {
    public static final double EPS = 1e-12;

    public static boolean equals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean equals(double a, double b) {
        return equals(a, b, EPS);
    }}
