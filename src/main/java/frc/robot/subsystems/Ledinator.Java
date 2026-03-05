package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.utilities.GeometryUtil;


import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.LedState;


public class Ledinator extends SubsystemBase {
    public static LedState _currentLED = LedState.Idle;

    private static final int LeftSlotStart = 8;
    private static final int LeftSlotEnd = 15;

    private static final int RightSlotStart = 8;
    private static final int RightSlotEnd = 15;

    private static final int ModuleLStart = 0;
    private static final int ModuleLEnd = 7;
    
    private static final int ModuleRStart = 0;
    private static final int ModuleREnd = 7;

    private final CANdle m_candleL = new CANdle(1, CANBus.roboRIO());
    private final CANdle m_candleR = new CANdle(2, CANBus.roboRIO());

    // private LedState leftAnimationType = LedState.Idle;
    // private LedState rightAnimationType = LedState.Idle;



    public void Robot() {
        var CANdleConfig = new CANdleConfiguration();

        CANdleConfig.LED.StripType = StripTypeValue.RGB;
        CANdleConfig.LED.BrightnessScalar = .5;

        m_candleL.getConfigurator().apply(CANdleConfig);
        m_candleR.getConfigurator().apply(CANdleConfig);

                m_candleL.setControl(
            new StrobeAnimation(ModuleLStart, ModuleLEnd)
            .withColor(new RGBWColor(0, 255, 0))
        );
                m_candleR.setControl(
            new StrobeAnimation(ModuleRStart, ModuleREnd)
            .withColor(new RGBWColor(24, 155, 204))
        );
    }
        
    
    public void Party() {
        m_candleL.setControl(
            new RainbowAnimation(LeftSlotStart, LeftSlotEnd)
        );
        m_candleR.setControl(
            new RainbowAnimation(RightSlotStart, RightSlotEnd)
        );
    }

    public void Alliance() {
        if (GeometryUtil.isRedAlliance()) {
             m_candleL.setControl(
            new SolidColor(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(255, 0, 0))
        );
                m_candleR.setControl(
            new SolidColor(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(255, 0, 0))
        );
        } else {
                        m_candleL.setControl(
            new SolidColor(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(24, 155, 204))
        );
                m_candleR.setControl(
            new SolidColor(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(24, 155, 204))
        ); 
        }

    }

        public void Crew() {
        m_candleL.setControl(
            new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(0, 255, 0))
        );
                m_candleR.setControl(
            new StrobeAnimation(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(24, 155, 204))
        );
    }


        public void Staged() {
        m_candleL.setControl(
            new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(255, 255, 0))
        );
                m_candleR.setControl(
            new StrobeAnimation(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(255, 255, 0))
        );
    }

            public void Climbed() {
         m_candleL.setControl(
            new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(0, 255, 0))
        );
                m_candleR.setControl(
            new StrobeAnimation(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(0, 255, 0))
        );
    }

    public void Idled() {
       m_candleL.setControl(
            new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
            .withColor(new RGBWColor(255, 0, 0))
        );
                m_candleR.setControl(
            new StrobeAnimation(RightSlotStart, RightSlotEnd)
            .withColor(new RGBWColor(255, 0, 0))
        );
    }

    public void handleCurrentState()
    {
        switch(_currentLED) 
        {
            case Idle:
            Party();
            break;
            case Climb:
            Climbed();
            break;
            case Staged:
            Staged();
            break;
            case Alliance:
            Alliance();
            break;
            default:
            Idled();
            break;

        }
    }

    public void periodic()
    {
        handleCurrentState();
        SmartDashboard.putString("Desired LED Type", _currentLED.toString());
        // Party();
    }
    
}



