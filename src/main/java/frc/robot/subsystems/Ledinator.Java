package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix6.configs.CANdiConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.LedState;
import frc.configuration.FieldConstants;


public class Ledinator extends SubsystemBase {
    private LedState _currentState = LedState.Idle;
    private CANdle _CandleL;
    private CANdle _CandleR;
    private CANdiConfiguration config;

    private Animation Staged;
    private Animation Climb;
    private Animation CREW;
    private Animation Disco;
    private int numLEDS_L = 0;
    private int numLEDS_R = 0;

    
    public Ledinator()
    {
        _CandleL = new CANdle(Constants.LEDinatorConstants.LED_L);
        _CandleR = new CANdle(Constants.LEDinatorConstants.LED_R);

    Disco = new RainbowAnimation(.5, .5, numLEDS);
    Stage = new StrobeAnimation(255,255,0,0,.05, numLEDS);
    Climb = new StrobeAnimation(0,255,0,0,.05, numLEDS);


    public void handleCurrentState()
    {
        switch(_currentState) 
        {
            case Idle:
                
            break;
            case Climb:
                
            break;
            default:

            break;

        }
    }

    public void periodic()
    {
        handleCurrentState()
    }
    
    }


}