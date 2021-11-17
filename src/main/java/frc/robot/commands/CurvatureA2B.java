package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Line;
import frc.robot.utils.Vector2;

public class CurvatureA2B extends CommandBase {

    DriveSubsystem drivetrain;
    Supplier<Vector2> goalSupplier;
    PIDController pid;
    double fwd;
    double tolerance = 0.1;

    public CurvatureA2B(Supplier<Vector2> goalSupplier, PIDController pid, double fwd, DriveSubsystem drivetrain) {
        addRequirements(drivetrain);
        this.goalSupplier = goalSupplier;
        this.pid = pid;
        this.fwd = fwd;
    }
    public CurvatureA2B(Supplier<Vector2> goalSupplier, PIDController pid, double fwd, double tolerance, DriveSubsystem drivetrain) {
        addRequirements(drivetrain);
        this.goalSupplier = goalSupplier;
        this.pid = pid;
        this.fwd = fwd;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Vector2 pos = new Vector2(drivetrain.getPose());
        double heading = drivetrain.getPose().getRotation().getRadians();

        // transform goal to local space, probably could use matrices but whatever
        Vector2 goal = goalSupplier.get();
        Vector2 relativeGoal = goal.sub(pos);
        Vector2 localGoal = new Vector2(
            Math.cos(-heading) * relativeGoal.x - Math.sin(-heading) * relativeGoal.y, 
            Math.cos(-heading) * relativeGoal.y + Math.sin(-heading) * relativeGoal.x
        );

        double leftRate = drivetrain.getLeftEncoder().getRate();
        double rightRate = drivetrain.getRightEncoder().getRate();
        double measuredCurvature = Units.metersToFeet(DriveConstants.kTrackwidthMeters) * (rightRate - leftRate) / (leftRate + rightRate);
        
        // convert heading to general form
        Line headingLine = new Line(Math.tan(heading), 0);
        double a = headingLine.getSlope();
        double b = -1;
        double c = headingLine.getYIntercept();

        // radius of circle to goal
        double radius = Math.abs((a * localGoal.x) + (b * localGoal.y) + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));

        double turn = pid.calculate(measuredCurvature, 1 / radius);

        // display bc am stupid
        System.out.println(
            "ideal: " + Math.round(radius) +
            "  |  measured: " + Math.round(1/measuredCurvature)+
            "  |  out: " + turn
        );

        drivetrain.arcadeDrive(0.4, -turn);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return new Vector2(drivetrain.getPose()).distanceTo(goalSupplier.get()) < tolerance;
    }
}
