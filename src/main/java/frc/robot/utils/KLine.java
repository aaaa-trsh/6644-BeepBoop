package frc.robot.utils;

public class KLine {
    public double slope;
    public double yIntercept;

    public KLine(KVector2 p1, KVector2 p2) {
        this.slope = (p2.y - p1.y) / (p2.x - p1.x);
        this.yIntercept = p1.y - (this.slope * p1.x);
    }

    public KLine(KVector2 p, double slope) {
        this.slope = slope;
        this.yIntercept = p.y - (this.slope * p.x);
    }

    public KLine(double slope, double yIntercept) {
        this.slope = slope;
        this.yIntercept = yIntercept;
    }

    public static KLine perpendicularBisector(KVector2 p1, KVector2 p2) {
        KVector2 p = KVector2.lerp(p1, p2, 0.5);
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double slope = -dx / dy;
        return new KLine(p, slope);
    }

    public KVector2 getIntersection(KLine line) {
        double x = (line.yIntercept - this.yIntercept) / (this.slope - line.slope);
        double y = this.slope * x + this.yIntercept;
        return new KVector2(x, y);
    }
}