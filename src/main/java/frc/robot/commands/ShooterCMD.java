package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Robot.*;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 *
 */
public class ShooterCMD extends Command {

    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final boolean fire;
    private final double speed;

    public ShooterCMD(ShooterSubsystem subsystem, IntakeSubsystem subsystem2, boolean fire, double speed) {

        m_shooter = subsystem;
        m_intake = subsystem2;
        this.fire = fire;
        this.speed = .5;
        addRequirements(m_shooter);
        addRequirements(m_intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ShooterSubsystem.shoot(speed, fire);
        Timer.delay(.7);
        IntakeSubsystem.intake(.5, fire);
        Timer.delay(.5);
        ShooterSubsystem.shoot(0, false);
        IntakeSubsystem.intake(0, false);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Intake", "Empty");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Timer.delay(1.2);
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
