package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.ShooterSubsystem;


/**
 *
 */
public class ShooterCMD2 extends Command {

    private final ShooterSubsystem m_shooter;
    private final boolean fire;
    private final double speed;

    public ShooterCMD2(ShooterSubsystem subsystem, boolean fire, double speed) {

        m_shooter = subsystem;
        this.fire = fire;
        this.speed = speed;
        addRequirements(m_shooter);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ShooterSubsystem.shoot(speed, fire);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Intake", "Empty");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
