package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class GyroResetCMD extends Command {

    private final DriveSubsystem m_robotDrive;
    private final double Heading;

    public GyroResetCMD(DriveSubsystem subsystem, double Heading) {
        this.Heading = Heading;
        m_robotDrive = subsystem;
        addRequirements(m_robotDrive);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        DriveSubsystem.setHeading(Heading);
    }
}
