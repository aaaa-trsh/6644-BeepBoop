package frc.robot.utils;

public class Line {
    public double slope;
    public double yIntercept;

    public Line(Vector2 p1, Vector2 p2) {
        this.slope = (p2.y - p1.y) / (p2.x - p1.x);
        this.yIntercept = p1.y - (this.slope * p1.x);
    }

    public Line(Vector2 p, double slope) {
        this.slope = slope;
        this.yIntercept = p.y - (this.slope * p.x);
    }

    public Line(double slope, double yIntercept) {
        this.slope = slope;
        this.yIntercept = yIntercept;
    }

    public static Line perpendicularBisector(Vector2 p1, Vector2 p2) {
        Vector2 p = Vector2.lerp(p1, p2, 0.5);
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double slope = -dx / dy;
        return new Line(p, slope);
    }

    public Vector2 getIntersection(Line line) {
        double x = (line.yIntercept - this.yIntercept) / (this.slope - line.slope);
        double y = this.slope * x + this.yIntercept;
        return new Vector2(x, y);
    }

    public String toString() {
        return "Line(slope=" + slope + ", y-int=" + yIntercept + ")";
    }
}