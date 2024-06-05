package frc.robot.subsystems;


import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase;



/**
 *
 */
public class ShooterSubsystem extends SubsystemBase {
    
    private static CANSparkFlex sparkFLEX10 = new CANSparkFlex(shooterConstants.shooterLowerCanID, CANSparkLowLevel.MotorType.kBrushless);

    private static CANSparkFlex sparkFLEX12 = new CANSparkFlex(shooterConstants.shooterTopCanID, CANSparkLowLevel.MotorType.kBrushless);
 
    public double speed;
    public boolean fire;
    
    
    /**
    *
    */
    public ShooterSubsystem() {
        
        sparkFLEX10.restoreFactoryDefaults();
        sparkFLEX10.setInverted(false);
        sparkFLEX10.setIdleMode(IdleMode.kCoast);
        sparkFLEX10.setSmartCurrentLimit(shooterConstants.shooter_top_limit);
        sparkFLEX10.burnFlash();
  

        
        sparkFLEX12.restoreFactoryDefaults();
        sparkFLEX12.setInverted(false);
        sparkFLEX12.setIdleMode(IdleMode.kCoast);
        sparkFLEX12.setSmartCurrentLimit(shooterConstants.shooter_lower_limit);
        sparkFLEX12.burnFlash();
        
        

    }

    public static void shoot(double speed, boolean fire) {
        if (fire) {
            sparkFLEX10.set(-1 * speed);  
            sparkFLEX12.set(speed);
        } else {
            sparkFLEX10.set(0);
            sparkFLEX12.set(0);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

