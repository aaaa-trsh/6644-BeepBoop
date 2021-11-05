package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class KVector2 {
    double x;
    double y;

    public KVector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public KVector2(Pose2d pose) {
        this.x = pose.getTranslation().getX();
        this.y = pose.getTranslation().getY();
    }

    static KVector2 lerp(KVector2 a, KVector2 b, double t) {
        return new KVector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    }


    public KVector2 add(KVector2 other) {
        return new KVector2(x + other.x, y + other.y);
    }

    public KVector2 sub(KVector2 other) {
        return new KVector2(x - other.x, y - other.y);
    }

    public KVector2 mul(double scalar) {
        return new KVector2(x * scalar, y * scalar);
    }

    public KVector2 div(double scalar) {
        return new KVector2(x / scalar, y / scalar);
    }

    public double dot(KVector2 other) {
        return x * other.x + y * other.y;
    }

    public double cross(KVector2 other) {
        return x * other.y - y * other.x;
    }

    public double len() {
        return Math.sqrt(x * x + y * y);
    }

    public KVector2 norm() {
        return div(len());
    }

    public KVector2 rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new KVector2(x * cos - y * sin, x * sin + y * cos);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public double angleTo(KVector2 other) {
        return Math.atan2(other.y - y, other.x - x);
    }

    public double distanceTo(KVector2 other) {
        return sub(other).len();
    }
}