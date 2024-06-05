package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmPIDCMD extends Command {
    private PIDController m_ArmPidController;
    private final ArmSubsystem m_arm_up;
    private double setPoint;
    private boolean m_UseCurrentPosition;
    private double speed;

    public ArmPIDCMD(ArmSubsystem m_arm_up, double setPoint, boolean UseCurrentPosition, double IZone) {
        this.m_arm_up = m_arm_up;
        m_ArmPidController = new PIDController(ArmConstants.kP, ArmConstants.kI, 0);
        m_ArmPidController.setTolerance(1);
        m_ArmPidController.setIZone(IZone);
        this.setPoint = setPoint;
        this.m_UseCurrentPosition = UseCurrentPosition;
        addRequirements(m_arm_up);

    }

    @Override
    public void initialize() {
        if (m_UseCurrentPosition) {
            setPoint = m_arm_up.getPivotEncoder();
        }
        SmartDashboard.putNumber("SET POINT", setPoint);
    }

    @Override
    public void execute() {
        double feedforward = 0.00;
        speed = m_ArmPidController.calculate(m_arm_up.getPivotEncoder(), setPoint);
        speed = (speed > 0) ? speed + feedforward : speed - feedforward;
        m_arm_up.armPIDSetSpeed(speed);
        if (m_ArmPidController.atSetpoint()) {
            ArmSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        if(m_ArmPidController.atSetpoint()) {
            return true;
        } else {
            return false;
        }
    }

    public void setPoint(double setPoint) {
        this.setPoint = setPoint;
    }
}
