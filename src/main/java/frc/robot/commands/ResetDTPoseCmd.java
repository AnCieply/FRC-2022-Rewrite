package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class ResetDTPoseCmd extends CommandBase {
    private final DriveTrain driveTrain;

    public ResetDTPoseCmd(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetOdometry(new Pose2d());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
