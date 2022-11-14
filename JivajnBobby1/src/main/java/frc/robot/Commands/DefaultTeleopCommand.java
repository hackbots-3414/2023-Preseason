package frc.robot.Commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain; 


public class DefaultTeleopCommand extends CommandBase {
    private static final Logger LOG = LoggerFactory.getLogger(DefaultTeleopCommand.class);
    private Drivetrain driveTrain;
    /**Creates a new DefaultTeleopCommand. */
    public DefaultTeleopCommand(Drivetrain drvTrain) {
        driveTrain = drvTrain;
        addRequirements(driveTrain);
    }


// Called when the command is initially scheduled.
@Override
public void initialize() {

}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() { 
    double xSpeed = RobotContainer.getInstance().getGamePad().getLeftY();
    double zRotation = RobotContainer.getInstance().getGamePad().getRightX();
    driveTrain.drive(xSpeed, zRotation);
    LOG.trace("execute(): xSpeed: {}, zRotation: {}" , xSpeed, zRotation); 















}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {

}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return false;
}
}
