package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.limelightInterface;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmAutoAim extends Command {
    private PIDController m_ArmPidController;
    private final ArmSubsystem m_arm_up;
    private double setPoint;
    private boolean m_UseCurrentPosition;
    private double speed;
    private double tY;
    private double cameraHeight;
    private double distance;
    private double armOut;
    private double armCurrent;
    private double armNew;
    private double targetHeight = 6.66777777;//ft
    private double maxHeight = 0;
    private double tagHeight = 4 +(3 + 7 / 8 + 4.5) / 12 ;//ft
    private double cameraOffset = 24;//degrees
    private double shooterAngle;
    private double pivotHeight = 1;//ft
    private double armLength = 19.75 / 12;//ft
    private double Velocity = 41.5;//ft/sec
    private double VelocityX = 0;
    private double VelocityY = 0;
    private double gravity = -32;//ft/sec

    public ArmAutoAim(ArmSubsystem m_arm_up, boolean UseCurrentPosition, double IZone) {
        this.m_arm_up = m_arm_up;
        m_ArmPidController = new PIDController(ArmConstants.kP, ArmConstants.kI, 0);
        m_ArmPidController.setTolerance(2);
        m_ArmPidController.setIZone(IZone);
        this.setPoint = armOut;
        this.m_UseCurrentPosition = UseCurrentPosition;
        addRequirements(m_arm_up);

    }

    @Override
    public void initialize() {
        if (m_UseCurrentPosition) {
            setPoint = m_arm_up.getPivotEncoder();
        }
    }

    @Override
    public void execute() {
        if (limelightInterface.hasValidTarget()) {
            tY = limelightInterface.getYOffset();
            armCurrent = m_arm_up.getPivotEncoder();
            shooterAngle = 270- (cameraOffset + armCurrent + 90);
            cameraHeight = pivotHeight + armLength * Math.sin((armCurrent - 90) * Math.PI / 180);
            distance = (tagHeight - cameraHeight) / Math.tan((shooterAngle + tY) * Math.PI / 180);
            armNew = Math.atan((targetHeight - cameraHeight) / distance) * 180 / Math.PI;
            armOut = armNew + 90;

            this.setPoint = armOut;

            SmartDashboard.putNumber("SET POINT", setPoint);
            SmartDashboard.putNumber("Y offset", tY);
            SmartDashboard.putNumber("Arm angle", shooterAngle);
            SmartDashboard.putNumber("Camera height", cameraHeight);
            SmartDashboard.putNumber("distance from goal", distance);
            SmartDashboard.putNumber("Final angle", armOut);

            double feedforward = 0.00;
            speed = m_ArmPidController.calculate(m_arm_up.getPivotEncoder(), setPoint);
            speed = (speed > 0) ? speed + feedforward : speed - feedforward;
            m_arm_up.armPIDSetSpeed(speed);
            if (m_ArmPidController.atSetpoint()) {
                ArmSubsystem.stop();
            }
        }
        else {
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
        armOut = setPoint;
    }
}
