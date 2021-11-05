package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CircleA2BCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.KVector2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        
        /*table.addEntryListener("path", (tb, k, e, v, f) -> {
            var cmdList = ((String)v.getValue()).split(" ");
            System.out.println((String)v.getValue());
            // int i = 0;
            CommandScheduler.getInstance().cancelAll();
            // CommandScheduler.getInstance().cancelAll();

            // double angle = Double.parseDouble(cmdList[i].split(",")[0]);
            var x = new SequentialCommandGroup();

            for (int i = 0; i < cmdList.length; i++) {
                double angle = Double.parseDouble(cmdList[i].split(",")[1]);
                var pid = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
                pid.setTolerance(3);
                x.addCommands(
                    new PIDCommand(
                        pid,
                        drivebase::getGyroAngle,
                        angle,
                        (output)->{
                            System.out.println(angle +" "+drivebase.getGyroAngle());
                            drivebase.arcadeDrive(0, output);
                        },
                        drivebase
                    ).withTimeout(30),
                    new WaitCommand(1)
                );
            }
            x.schedule();
                // new PIDCommand(
                //     new PIDController(
                //         DriveConstants.kTurnP,
                //         DriveConstants.kTurnI,
                //         DriveConstants.kTurnD
                //     ),
                //     drivebase::getGyroAngle,
                //     angle,
                //     (output)->{
                //         System.out.println(angle +" "+drivebase.getGyroAngle());
                //         drivebase.arcadeDrive(0, output);
                //     },
                //     drivebase
                // )
            //);            
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        */
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
        // return new PIDCommand(
        //     new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        //     drivebase::getGyroAngle,
        //     90,
        //     (output)->{
        //         System.out.println(90 +" "+drivebase.getGyroAngle());
        //         drivebase.arcadeDrive(0, output);
        //     },
        //     drivebase
        // );
        return new CircleA2BCommand(
            () -> new KVector2(drivebase.getPose()),
            () -> drivebase.getPose().getRotation(),
            () -> new KVector2(-1, 0),
            new PIDController(80, 0, 0),
            3,
            drivebase.getKinematics(),
            (l, r) -> drivebase.tankDriveVolts(l, r),
            drivebase
        );
        // var v = table.getEntry("pos");
        // var cmdList = ((String)v.getValue()).split(" ");
        // System.out.println(cmdList);
        // int i = 0;
        // CommandScheduler.getInstance().cancelAll();
        // new PIDCommand(
        //     new PIDController(1, DriveConstants.kTurnI, DriveConstants.kTurnD),
        //     ()->drivebase.getPose().getRotation().getDegrees(),
        //     (output)->{drivebase.arcadeDrive(0, output);},
        //     drivebase
        // ).schedule();
        // double angle = Double.parseDouble(cmdList[i].split(",")[0]);
        // new PIDCommand(
        //     new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        //     drivebase::getGyroAngle,
        //     angle,
        //     (output)->{
        //         System.out.println(angle +" "+drivebase.getGyroAngle());
        //         drivebase.arcadeDrive(0, output);
        //     },
        //     null
        // ).schedule();
        // posEntry.setNumberArray(value)
        // return new PIDCommand(
        //     new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        //     drivebase::getGyroAngle,
        //     90,
        //     (output)->{
        //         System.out.println(90 +" "+drivebase.getGyroAngle());
        //         drivebase.arcadeDrive(0, output);
        //     },
        //     drivebase
        // );
    }
}
