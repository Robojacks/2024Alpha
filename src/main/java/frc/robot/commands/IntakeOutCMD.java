package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


/**
 *
 */
public class IntakeOutCMD extends Command {

        private final IntakeSubsystem m_intake;
        private final boolean in;



    public IntakeOutCMD(IntakeSubsystem subsystem, boolean in) {

        this.in = in;
        m_intake = subsystem;
        addRequirements(m_intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        IntakeSubsystem.intake(-1 * intakeConstants.intakeSpeed, in);
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

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
