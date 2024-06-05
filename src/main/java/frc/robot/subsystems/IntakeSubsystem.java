package frc.robot.subsystems;


import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;



/**
 *
 */
public class IntakeSubsystem extends SubsystemBase {


    private static CANSparkMax sparkMAX11 = new CANSparkMax(intakeConstants.intakeCanID, CANSparkLowLevel.MotorType.kBrushless);
    boolean prevState;
    public double speed;
    public final static DigitalInput m_beam_break = new DigitalInput(0);
    //public static SparkLimitSwitch m_beam_break = sparkMAX11.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    
    /**
    *
    */
    public IntakeSubsystem(double speed) {
        
        sparkMAX11.restoreFactoryDefaults();
        sparkMAX11.setInverted(true);
        sparkMAX11.setIdleMode(IdleMode.kBrake);
        sparkMAX11.setSmartCurrentLimit(intakeConstants.intake_limit);
        sparkMAX11.burnFlash();
        

       prevState = IntakeSubsystem.m_beam_break.get();
        recordState(prevState);
    }

   public void recordState(boolean isLimitClicked) 
   {
    prevState = isLimitClicked;
    String currenStateMsg = (isLimitClicked) ? "NOTE" : "NO NOTE";
       DriverStation.reportWarning("BeamBreaker update " + currenStateMsg, false);
   }

    public static void intake(double speed, boolean in) {
        if (in) {
            sparkMAX11.set(speed);
        } else {
            sparkMAX11.set(0);
        }
    }
    @Override
    public void periodic() {

       boolean currentState = IntakeSubsystem.m_beam_break.get();
       if (prevState != currentState)
                recordState(currentState);


        // This method will be called once per scheduler run  
        //sparkMAX11.set(speed);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

