package frc.robot;

import java.util.List;

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
import frc.robot.subsystems.*;
import frc.robot.utils.Vector2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    DriveSubsystem drivebase = new DriveSubsystem();
    Compressor compressor = new Compressor();
    Joystick joystick = new Joystick(0);

    NetworkTableEntry posEntry;
    NetworkTable table;
    public RobotContainer() {
        compressor.start();
        configureButtonBindings();

        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        table = ntInstance.getTable("testws");
        posEntry = table.getEntry("pos");
    }

    private void configureButtonBindings() {
        drivebase.setDefaultCommand(new RunCommand(() -> drivebase.arcadeDrive(joystick.getY(), joystick.getX()), drivebase));
        JoystickButton button = new JoystickButton(joystick, 6);
        button.whenPressed(new InstantCommand(()->drivebase.setTransmission(true)))
            .whenReleased(new InstantCommand(()->drivebase.setTransmission(false)));
    }
    
    public void periodic() {
        posEntry.setDoubleArray(new double[]{ Units.metersToFeet(drivebase.getPose().getX()), Units.metersToFeet(drivebase.getPose().getY()), drivebase.getPose().getRotation().getRadians() });
    }

    public Command ramseteCommand(String trajectoryJSON) {
        TrajectoryConfig config = new TrajectoryConfig(.5, 10);
        config.setKinematics(drivebase.getKinematics());
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );

        drivebase.resetOdometry(trajectory.getInitialPose());

        RamseteCommand command = new RamseteCommand(
            trajectory,
            drivebase::getPose,
            new RamseteController(2, .7),
            drivebase.getFeedforward(),
            drivebase.getKinematics(),
            drivebase::getSpeeds,
            drivebase.getLeftPIDController(),
            drivebase.getRightPIDController(),
            drivebase::tankDriveVolts,
            drivebase
        );
    
        return command.andThen(() -> drivebase.tankDriveVolts(0, 0))
                      .andThen(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kTurnP,
                    DriveConstants.kTurnI,
                    DriveConstants.kTurnD
                ),
                drivebase::getGyroAngle,
                0,
                (x) -> {
                    drivebase.arcadeDrive(0, x);
                },
                drivebase
            )
        );
    }

    public Command getAutonomousCommand()
    {
        return new CircleA2BCommand(
            () -> new Vector2(drivebase.getPose()),
            () -> drivebase.getPose().getRotation(),
            () -> new Vector2(1, 1),
            new PIDController(80, 0, 0),
            .01,
            drivebase.getKinematics(),
            (l, r) -> drivebase.tankDriveVolts(l, r),
            drivebase
        );
    }
}
