package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CircleA2BCommand;
import frc.robot.commands.DoubleDrivePID;
import frc.robot.subsystems.*;
import frc.robot.utils.Vector2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    DriveSubsystem drivebase = new DriveSubsystem();
    Compressor compressor = new Compressor();
    Joystick joystick = new Joystick(0);

    NetworkTableEntry posEntry;
    NetworkTableEntry pathEntry;
    NetworkTable table;

    String curPathString = "";

    public RobotContainer() {
        compressor.start();
        configureButtonBindings();

        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        table = ntInstance.getTable("testws");
        posEntry = table.getEntry("pos");
        pathEntry = table.getEntry("path");
        curPathString = pathEntry.getValue().getString();
        table.addEntryListener("path", (table, key, entry, value, flags) -> {
            if (!curPathString.contains(value.getString())) {
                System.out.println("New path: " + value.getString());
                curPathString = value.getString();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void configureButtonBindings() {
        drivebase.setDefaultCommand(new RunCommand(() -> {
            drivebase.arcadeDrive(joystick.getY(), joystick.getX());
        }, drivebase));

        JoystickButton button = new JoystickButton(joystick, 6);
        button.whenPressed(new InstantCommand(()->drivebase.setTransmission(true)))
            .whenReleased(new InstantCommand(()->drivebase.setTransmission(false)));
    }
    
    public void periodic() {
        posEntry.setDoubleArray(new double[]{ Units.metersToFeet(drivebase.getPose().getX()), Units.metersToFeet(drivebase.getPose().getY()), drivebase.getPose().getRotation().getRadians() });
    }

    public List<Translation2d> waypointsFromPathString(String pathString) {
        if (pathString.length() == 0) return new ArrayList<Translation2d>();
        List<Translation2d> waypoints = new ArrayList<Translation2d>();
        String[] pointStrings = pathString.trim().split("\\s+");

        for (int i = 0; i < pointStrings.length; i++) {
            System.out.println(pointStrings[i]);
            String[] coordinates = pointStrings[i].split(",");
            // string to double
            double x = -Units.feetToMeters(Double.parseDouble(coordinates[0]));
            double y = -Units.feetToMeters(Double.parseDouble(coordinates[1]));
            waypoints.add(new Translation2d(x, y));
        }
        return waypoints;
    }
    public Command ramseteCommand(String pathString) {
        TrajectoryConfig config = new TrajectoryConfig(.7, 10);
        config.setKinematics(drivebase.getKinematics());
        
        List<Translation2d> waypoints = waypointsFromPathString(pathString);
        Translation2d end = waypoints.remove(waypoints.size() - 1);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            drivebase.getPose(),
            waypoints,
            new Pose2d(end.getX(), end.getY(), Rotation2d.fromDegrees(0)),
            config
        );


        drivebase.resetOdometry(trajectory.getInitialPose());
        var controller = new RamseteController(6, .7);
        // controller.setEnabled(false);
        RamseteCommand command = new RamseteCommand(
            trajectory,
            drivebase::getPose,
            controller,
            drivebase.getFeedforward(),
            drivebase.getKinematics(),
            drivebase::getSpeeds,
            drivebase.getLeftPIDController(),
            drivebase.getRightPIDController(),
            drivebase::tankDriveVolts,
            drivebase
        );
    
        return command.andThen(() -> drivebase.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand()
    {
        return ramseteCommand(curPathString);
    }
}
