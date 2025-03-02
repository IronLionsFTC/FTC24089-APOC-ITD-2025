package core.math;

import com.pedropathing.localization.Pose;

public class Vector {
    public double x;
    public double y;
    public double d;
    public double m;

    public Vector(double x, double y, double d, double m) {
        this.x = x;
        this.y = y;
        this.d = d;
        this.m = m;
    }

    public static Vector cartesian(double x, double y) {
        return new Vector(x, y, Math.atan2(y, x), Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    public static Vector polar(double d, double m) {
        return new Vector(Math.cos(d) * m, Math.sin(d) * m, d, m);
    }

    public Pose pose(double rotation) { return new Pose(this.x, this.y, rotation); }
    public Vector add(Vector other) { return Vector.cartesian(this.x + other.x, this.y + other.y); }
    public Vector sub(Vector other) { return Vector.cartesian(this.x - other.x, this.y - other.y); }
    public Vector mul(double other) { return Vector.cartesian(this.x * other, this.y * other); }
    public Vector div(double other) { return Vector.cartesian(this.x / other, this.y / other); }
    public double dot(Vector other) { return this.x * other.x + this.y *  other.y; }
}
