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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.LedState;

public class Ledinator extends SubsystemBase {
        public static LedState _currentLED = LedState.IDLE;

        private static final int LeftSlotStart = 0;
        private static final int LeftSlotEnd = 30;

        private static final int RightSlotStart = 0;
        private static final int RightSlotEnd = 30;

        private static final int ModuleLStart = 0;
        private static final int ModuleLEnd = 20;

        private static final int ModuleRStart = 0;
        private static final int ModuleREnd = 20;

        private final CANdle m_candleL = new CANdle(30, CANBus.roboRIO());
        private final CANdle m_candleR = new CANdle(31, CANBus.roboRIO());

        // private LedState leftAnimationType = LedState.Idle;
        // private LedState rightAnimationType = LedState.Idle;

        public void Robot() {
                var CANdleConfig = new CANdleConfiguration();

                CANdleConfig.LED.StripType = StripTypeValue.RGB;

                m_candleL.getConfigurator().apply(CANdleConfig);
                m_candleR.getConfigurator().apply(CANdleConfig);

                m_candleL.setControl(
                                new StrobeAnimation(ModuleLStart, ModuleLEnd)
                                                .withColor(new RGBWColor(0, 255, 0)));
                m_candleR.setControl(
                                new StrobeAnimation(ModuleRStart, ModuleREnd)
                                                .withColor(new RGBWColor(100, 0, 255)));
        }

        public void party() {
                m_candleL.setControl(
                                new RainbowAnimation(LeftSlotStart, LeftSlotEnd));
                m_candleR.setControl(
                                new RainbowAnimation(RightSlotStart, RightSlotEnd));
        }

        public void alliance() {
                if (GeometryUtil.isRedAlliance()) {
                        m_candleL.setControl(
                                        new SolidColor(LeftSlotStart, LeftSlotEnd)
                                                        .withColor(new RGBWColor(255, 0, 0)));
                        m_candleR.setControl(
                                        new SolidColor(RightSlotStart, RightSlotEnd)
                                                        .withColor(new RGBWColor(255, 0, 0)));
                } else {
                        m_candleL.setControl(
                                        new SolidColor(LeftSlotStart, LeftSlotEnd)
                                                        .withColor(new RGBWColor(0, 0, 255)));
                        m_candleR.setControl(
                                        new SolidColor(RightSlotStart, RightSlotEnd)
                                                        .withColor(new RGBWColor(0, 0, 255)));
                }

        }

        public void crew() {
                m_candleL.setControl(
                                new ColorFlowAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(0, 255, 0)).withFrameRate(5));
                m_candleR.setControl(
                                new ColorFlowAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(0, 0, 255)).withFrameRate(5));
        }

        public void staged() {
                m_candleL.setControl(
                                new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(255, 255, 255))
                                                .withFrameRate(5));
                m_candleR.setControl(
                                new StrobeAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(255, 255, 255))
                                                .withFrameRate(5));
        }

        public void shoot() {
                m_candleL.setControl(
                                new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(0, 0, 255))
                                                .withFrameRate(5));
                m_candleR.setControl(
                                new StrobeAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(0, 0, 255))
                                                .withFrameRate(5));
        }

        public void climbed() {
                m_candleL.setControl(
                                new ColorFlowAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(0, 255, 0)));
                m_candleR.setControl(
                                new ColorFlowAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(0, 255, 0)));
        }

        public void idled() {
                m_candleL.setControl(
                                new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(255, 0, 0))
                                                .withFrameRate(5));
                m_candleR.setControl(
                                new StrobeAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(255, 0, 0))
                                                .withFrameRate(5));
        }

        public void warning() {
                m_candleL.setControl(
                                new StrobeAnimation(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(255, 255, 0))
                                                .withFrameRate(5));
                m_candleR.setControl(
                                new StrobeAnimation(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(255, 255, 0))
                                                .withFrameRate(5));
        }

        public void active() {
                m_candleL.setControl(
                                new SolidColor(LeftSlotStart, LeftSlotEnd)
                                                .withColor(new RGBWColor(255, 0, 255)));
                m_candleR.setControl(
                                new SolidColor(RightSlotStart, RightSlotEnd)
                                                .withColor(new RGBWColor(255, 0, 255)));
        }

        private void handleCurrentState() {
                switch (_currentLED) {
                        case IDLE:
                                crew();
                                break;
                        case CLIMB:
                                climbed();
                                break;
                        case STAGED:
                                staged();
                                break;
                        case ALLIANCE:
                                alliance();
                                break;
                        case SHOOT:
                                shoot();
                                break;
                        case INTAKE:
                                party();
                                break;
                        case WARNING:
                                warning();
                                break;
                        case ACTIVE:
                                active();
                                break;
                        default:
                                idled();
                                break;

                }
        }

        @Override
        public void periodic() {
                handleCurrentState();
        }

        public void setWantedState(LedState wantedState) {
                if (_currentLED != wantedState) {
                        _currentLED = wantedState;
                }
        }
}