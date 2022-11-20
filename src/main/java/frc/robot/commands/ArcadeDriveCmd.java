package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCmd extends CommandBase {
    // TODO: ** I should move this somewhere else hmmm. **
    // A DriveTrain variable here does not necessarily mean that a new drivetrain is created here.
    // When it comes to objects, different variables can point to the same object.
    // For example, if you created two Scanner variables, and set one to a new Scanner,
    // then set the second variable equal to the first, both of those would refer to the same Scanner.
    // It might be a bit confusing at first, but you'll get it.
    private final DriveTrain driveTrain;
    // I'm not necessarily sure about the purporse of suppliers yet.
    private final Supplier<Double> velocityFunc, rotationFunc;

    public ArcadeDriveCmd(DriveTrain driveTrain, Supplier<Double> velocityFunc, Supplier<Double> rotationFunc) {
        this.velocityFunc = velocityFunc;
        this.rotationFunc = rotationFunc;
        this.driveTrain = driveTrain;
        // Makes the DriveTrain subsystem a dependency of this command,
        // which means this command will only run when the DriveTrain is
        // not used being used by another command.
        addRequirements(driveTrain);
    }
    
    // The execute method is what the command actually does.
    // So, all thius command does is make the DriveTrain move.
    @Override
    public void execute() {
        double velocity = velocityFunc.get();
        double rotation = rotationFunc.get();
        driveTrain.setApplyArcadeDrive(velocity, rotation);
    }

    // Returns true when the command should end.
    // The above statement is only true with REGULAR commands.
    // This command is a DEFAULT command, meaning
    // it runs whenever no other command is scheduled
    // that requires the DriveTrain subsystem.
    // DEFAULT subsystems should always return FALSE.
    @Override
    public boolean isFinished() {
        return false;
    }
}
