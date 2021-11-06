package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Vector2(Pose2d pose) {
        this.x = pose.getTranslation().getX();
        this.y = pose.getTranslation().getY();
    }

    static Vector2 lerp(Vector2 a, Vector2 b, double t) {
        return new Vector2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    }


    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    public Vector2 sub(Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    public Vector2 mul(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }

    public Vector2 div(double scalar) {
        return new Vector2(x / scalar, y / scalar);
    }

    public double dot(Vector2 other) {
        return x * other.x + y * other.y;
    }

    public double cross(Vector2 other) {
        return x * other.y - y * other.x;
    }

    public double len() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2 norm() {
        return div(len());
    }

    public Vector2 rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2(x * cos - y * sin, x * sin + y * cos);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public double angleTo(Vector2 other) {
        return Math.atan2(other.y - y, other.x - x);
    }

    public double distanceTo(Vector2 other) {
        return sub(other).len();
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}