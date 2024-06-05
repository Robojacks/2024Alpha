package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


/**
 *
 */
public class ArmUpCMD extends Command {

        private final ArmSubsystem m_arm_up;
        private final boolean up;
        private final boolean down;

    public ArmUpCMD(ArmSubsystem subsystem, boolean up, boolean down) {

        this.up = up;
        this.down = down;
        m_arm_up = subsystem;
        addRequirements(m_arm_up);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ArmSubsystem.arm(ArmConstants.armSpeed, up, down);
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
