package frc.robot.commands;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.revrobotics.SparkLimitSwitch;

import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


/**
 *
 */
public class IntakeCMD_NoBeamBreak extends Command {

        private final IntakeSubsystem m_intake;
        private final boolean in;
        //private final DigitalInput m_beam_break = new DigitalInput(0);

    public IntakeCMD_NoBeamBreak(IntakeSubsystem subsystem, boolean in) {

        this.in = in;
        m_intake = subsystem;
        addRequirements(m_intake);
        SmartDashboard.putString("Intake", "Empty");

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Intake", "Empty");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        IntakeSubsystem.intake(intakeConstants.intakeSpeed, in);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.intake(0, in);
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
