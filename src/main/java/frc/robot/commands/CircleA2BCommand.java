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
import frc.robot.utils.KLine;
import frc.robot.utils.KVector2;

public class CircleA2BCommand extends CommandBase {
    private Supplier<KVector2> poseSupplier;
    private Supplier<Rotation2d> rotationSupplier;
    private Supplier<KVector2> goalSupplier;
    private PIDController wheelPIDController;
    private double velocity;
    private DifferentialDriveKinematics kinematics;
    private DriveSubsystem drivetrain;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private BiConsumer<Double, Double> outputVolts;

    public CircleA2BCommand(
        Supplier<KVector2> startSupplier, 
        Supplier<Rotation2d> rotationSupplier, 
        Supplier<KVector2> goalSupplier,
        PIDController wheelPIDController,
        double velocity,
        DifferentialDriveKinematics kinematics,
        BiConsumer<Double, Double> outputVolts,
        DriveSubsystem drivetrain
    ) {
        addRequirements(drivetrain);
        this.poseSupplier = startSupplier;
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
        double radius = new KLine(poseSupplier.get(), Math.tan(rotationSupplier.get().getRadians())).getIntersection(
            KLine.perpendicularBisector(poseSupplier.get(), goalSupplier.get())
        ).distanceTo(poseSupplier.get());
        System.out.println("rad: "+radius);
        double angularVelocity = velocity / radius;
        
        // var chassisSpeeds = new ChassisSpeeds(velocity, 0, angularVelocity);
        var chassisSpeeds = new ChassisSpeeds(1, 0, Math.PI/2);
        wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftVelocity = wheelSpeeds.leftMetersPerSecond;
        double rightVelocity = wheelSpeeds.rightMetersPerSecond;
        double leftVolts = wheelPIDController.calculate(drivetrain.getLeftEncoder().getRate(), leftVelocity);
        double rightVolts = wheelPIDController.calculate(drivetrain.getRightEncoder().getRate(), rightVelocity);
        outputVolts.accept(leftVolts, rightVolts);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
