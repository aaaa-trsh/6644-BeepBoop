// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DoubleDrivePID extends CommandBase {
    double leftSpeed = 0;
    double rightSpeed = 0;
    PIDController controller;
    DriveSubsystem drivetrain;
    public DoubleDrivePID(double leftSpeed, double rightSpeed, PIDController controller, DriveSubsystem drivetrain) {
        addRequirements(drivetrain);
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.controller = controller;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double leftMeasured = Units.feetToMeters(drivetrain.getLeftEncoder().getRate()),
            rightMeasured = Units.feetToMeters(drivetrain.getRightEncoder().getRate());

        double left = controller.calculate(leftMeasured, leftSpeed), 
            right = controller.calculate(rightMeasured, rightSpeed);
        
        System.out.println(
            "ideal: " + Math.round(Units.metersToInches((DriveConstants.kTrackwidthMeters / 2) * (leftSpeed + rightSpeed) / (rightSpeed - leftSpeed))) +
            "  |  measured: " + Math.round(Units.metersToInches((DriveConstants.kTrackwidthMeters / 2) * (leftMeasured + rightMeasured) / (rightMeasured - leftMeasured)))
        );
        drivetrain.tankDriveVolts(left * 12, right * 12);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
