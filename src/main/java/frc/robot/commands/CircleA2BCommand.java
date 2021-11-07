package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
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
    int i = 0;
    @Override
    public void execute() {
        i += 1;
        // FOR SIMULATION
        // var heading = simHeading;
        // var pos = new Vector2(simX, simY);

        var heading = rotationSupplier.get().getRadians();
        var pos = posSupplier.get();
        var goal = goalSupplier.get();

        var relativeGoal = goal.sub(pos);
        var localGoal = new Vector2(
            Math.cos(-heading) * relativeGoal.x - Math.sin(-heading) * relativeGoal.y, 
            Math.cos(-heading) * relativeGoal.y + Math.sin(-heading) * relativeGoal.x
        );

        var radius = Math.pow(localGoal.len(), 2) / (2 * localGoal.y);
        var angularVelocity = velocity / radius;
        var baseVel = angularVelocity * DriveConstants.kTrackwidthMeters;
        
        var left = wheelPIDController.calculate(drivetrain.getLeftEncoder().getRate(), angularVelocity + baseVel);
        var right = wheelPIDController.calculate(drivetrain.getLeftEncoder().getRate(), angularVelocity + baseVel);
        
        if (i % 5 == 0) {
            System.out.println(
                "goal at: " + localGoal + 
                " | distance: " + Math.round(localGoal.len() * 100.0)/100.0 + 
                " | curvature: " + Math.round(angularVelocity * 100.0)/100.0);
            i = 0;
        }
        
        outputVolts.accept(left, right);
    }

    double simX;
    double simY;
    double simHeading;
    void simUpdate(double velocity, double angularVelocity) {
        var d = velocity;
        var w = angularVelocity;
        var x = d * Math.cos(simHeading);
        var y = d * Math.sin(simHeading);
        simX += x;
        simY += y;
        simHeading += w;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
