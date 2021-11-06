package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Line;
import frc.robot.utils.Vector2;

public class CircleA2BCommand extends CommandBase {
    private Supplier<Vector2> posSupplier;
    private Supplier<Rotation2d> rotationSupplier;
    private Supplier<Vector2> goalSupplier;
    private PIDController wheelPIDController;
    private double velocity;
    private DifferentialDriveKinematics kinematics;
    private DriveSubsystem drivetrain;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private BiConsumer<Double, Double> outputVolts;

    public CircleA2BCommand(
        Supplier<Vector2> startSupplier, 
        Supplier<Rotation2d> rotationSupplier, 
        Supplier<Vector2> goalSupplier,
        PIDController wheelPIDController,
        double velocity,
        DifferentialDriveKinematics kinematics,
        BiConsumer<Double, Double> outputVolts,
        DriveSubsystem drivetrain
    ) {
        addRequirements(drivetrain);
        this.posSupplier = startSupplier;
        this.rotationSupplier = rotationSupplier;
        this.goalSupplier = goalSupplier;
        this.wheelPIDController = wheelPIDController;
        this.velocity = velocity;
        this.kinematics = kinematics;
        this.outputVolts = outputVolts;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var heading = rotationSupplier.get();
        var robotLine = new Line(posSupplier.get(), Math.tan(rotationSupplier.get().getRadians()));
        var abBisect = Line.perpendicularBisector(posSupplier.get(), goalSupplier.get());
        // System.out.println(abBisect);
        var center = robotLine.getIntersection(abBisect);
        var radius = center.distanceTo(posSupplier.get());
        var immediateGoal = goalSupplier.get();
        var relativeGoal = immediateGoal.sub(posSupplier.get());
        var localGoal = new Vector2(
            Math.cos(-heading.getRadians()) * relativeGoal.x - Math.sin(-heading.getRadians()) * relativeGoal.y,
            Math.sin(-heading.getRadians()) * relativeGoal.x + Math.cos(-heading.getRadians()) * relativeGoal.y
        );

        
        // System.out.println("rad: "+radius);
        // double angularVelocity = velocity / radius;
        
        // var chassisSpeeds = new ChassisSpeeds(velocity, 0, angularVelocity);
        var angularVelocity = localGoal.x / (radius * radius);
        var chassisSpeeds = new ChassisSpeeds(velocity, 0, angularVelocity);
        wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;
        double leftVolts = wheelPIDController.calculate(drivetrain.getLeftEncoder().getRate(), leftVelocity);
        double rightVolts = wheelPIDController.calculate(drivetrain.getRightEncoder().getRate(), rightVelocity);
        outputVolts.accept(leftVolts, rightVolts);
        System.out.println(posSupplier.get().distanceTo(immediateGoal) + " curvature:" + angularVelocity);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
