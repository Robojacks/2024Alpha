package frc.robot.subsystems;


import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/**
 *
 */
public class ArmSubsystem extends SubsystemBase {


    private static CANSparkMax sparkMAX5 = new CANSparkMax(ArmConstants.armCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax sparkMAX4 = new CANSparkMax(ArmConstants.arm2CANID, CANSparkLowLevel.MotorType.kBrushless);
    private static AbsoluteEncoder encoder;
    public double speed;
    public boolean down;
    public double rotation;

    /**
    *
    */
    public ArmSubsystem() {
 
        sparkMAX5.restoreFactoryDefaults();
        sparkMAX5.setIdleMode(IdleMode.kBrake);
        sparkMAX5.setSmartCurrentLimit(ArmConstants.arm_limit);
        sparkMAX5.setInverted(true);
        encoder = sparkMAX5.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);
        encoder.setInverted(false);
        sparkMAX5.burnFlash();

        sparkMAX4.restoreFactoryDefaults();
        sparkMAX4.follow(sparkMAX5);
        sparkMAX4.setIdleMode(IdleMode.kBrake);
        sparkMAX4.setSmartCurrentLimit(ArmConstants.arm_limit);
        sparkMAX4.burnFlash();
        /*
         * 
         * 
         */

    }

    public static void arm(double speed, boolean up, boolean down) {
        if (up) {
            sparkMAX5.set(speed);
        } else {
            if (down) {
                sparkMAX5.set(-1 * speed);
            } else {
                sparkMAX5.set(0);
            }
        }
    }

    public void armPIDSetSpeed(double speed) {
        sparkMAX5.set(speed);
    }

    public double getPivotEncoder() {
        return encoder.getPosition();
    }

    public static void  stop() {
        sparkMAX5.set(0);
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber("ARM POSOTION", encoder.getPosition());
       //sparkMAX5.getFault();
                // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

