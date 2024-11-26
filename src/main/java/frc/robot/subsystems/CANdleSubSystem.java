// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSubSystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(25, "rio");
    private final int LedCount = 35;
    public static int setAllRed = 0;
    public static int setAllGreen = 0;
    public static int setAllBlue = 0;
    public static int setAllWhite = 0;
    public static double setAllBrightness = 0;
    
    // private XboxController joystick;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    private AnimationTypes m_currentAnimation;

    public CANdleSubSystem() {
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    
    /**
     * Sets up a color flow animation with the specified RGBW values and speed.
     * 
     * @param red   The intensity of the red color (0-255).
     * @param green The intensity of the green color (0-255).
     * @param blue  The intensity of the blue color (0-255).
     * @param white The intensity of the white color (0-255).
     * @param speed The speed of the color flow animation (0-1).
     */
    public void ColorFlowAnimation(int red, int green, int blue, int white, double speed) {
        m_toAnimate = new ColorFlowAnimation(red, green, blue, white, speed, LedCount, Direction.Forward);
        // m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
    }
    /**
     * Sets up and starts a fire animation with the specified parameters.
     * 
     * @param brightness The brightness of the fire animation (0.0 - 1.0).
     * @param speed The speed of the fire animation (0.0 - 1.0).
     * @param cooling The cooling factor of the fire animation (0.0 - 1.0).
     * @param sparking The sparking factor of the fire animation (0.0 - 1.0).
     */
    public void FireAnimation(double brightness, double speed,  double cooling, double sparking) {
        m_toAnimate = new FireAnimation(brightness, speed, LedCount, sparking, cooling);
    }
    /**
     * Sets up and starts a Larson scanner animation with the specified parameters.
     * 
     * @param red The intensity of the red color (0-255).
     * @param green The intensity of the green color (0-255).
     * @param blue The intensity of the blue color (0-255).
     * @param white The intensity of the white color (0-255).
     * @param speed The speed of the animation (0-1).
     * @param bounceMode The bounce mode of the animation.
     * @param width The width of the scanner.
     */
    public void LarsonAnimation(int red, int green, int blue, int white, double speed, BounceMode bounceMode, int width) {
        m_toAnimate = new LarsonAnimation(red, green, blue, white, speed, LedCount, bounceMode, width);
    }
    /**
     * Sets up and starts a rainbow animation with the specified parameters.
     * 
     * @param brightness The brightness of the rainbow animation (0.0 - 1.0).
     * @param speed The speed of the rainbow animation (0.0 - 1.0).
     */
    public void RainbowAnimation(double brightness, double speed) {
        m_toAnimate = new RainbowAnimation(brightness, speed, LedCount);
    }
    /**
     * Sets up and starts an RGB fade animation with the specified parameters.
     * 
     * @param speed The speed of the fade animation (0.0 - 1.0).
     * @param brightness The brightness of the fade animation (0.0 - 1.0).
     */
    public void RgbFadeAnimation(double speed, double brightness) {
        m_toAnimate = new RgbFadeAnimation(speed, brightness, LedCount);
    }
    /**
     * Creates a single fade animation with the specified color and speed.
     * 
     * @param red   the intensity of the red color component (0-255)
     * @param green the intensity of the green color component (0-255)
     * @param blue  the intensity of the blue color component (0-255)
     * @param white the intensity of the white color component (0-255)
     * @param speed the speed of the animation (0-1)
     */
    public void SingleFadeAnimation(int red, int green, int blue, int white, double speed) {
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, LedCount);
    }
    /**
     * Sets up and starts a strobe animation with the specified parameters.
     * 
     * @param red   the intensity of the red color component (0-255)
     * @param green the intensity of the green color component (0-255)
     * @param blue  the intensity of the blue color component (0-255)
     * @param white the intensity of the white color component (0-255)
     * @param speed the speed of the animation (0-1)
     */
    public void StrobeAnimation(int red, int green, int blue, int white, double speed) {
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, LedCount);
    }
    /**
     * Sets up and starts a twinkle animation with the specified parameters.
     * 
     * @param red   the intensity of the red color component (0-255)
     * @param green the intensity of the green color component (0-255)
     * @param blue  the intensity of the blue color component (0-255)
     * @param white the intensity of the white color component (0-255)
     * @param speed the speed of the animation (0-1)
     * @param twinklePercent The percentage of LEDs to twinkle.
     */
    public void TwinkleAnimation(int red, int green, int blue, int white, double speed, TwinklePercent twinklePercent) {
        m_toAnimate = new TwinkleAnimation(red, green, blue, white, speed, LedCount, twinklePercent);
    }
    /**
     * Sets up and starts a twinkle off animation with the specified parameters.
     * 
     * @param red   the intensity of the red color component (0-255)
     * @param green the intensity of the green color component (0-255)
     * @param blue  the intensity of the blue color component (0-255)
     */
    public void TwinkleOffAnimation(int red, int green, int blue) {
        m_toAnimate = new TwinkleOffAnimation(red, green, blue);
    }
    /**
     * Sets all LEDs to the specified RGBW values.
     * 
     * @param red The intensity of the red color (0-255).
     * @param green The intensity of the green color (0-255).
     * @param blue The intensity of the blue color (0-255).
     * @param white The intensity of the white color (0-255).
     * @param brightness The brightness of the LEDs (0.0 - 1.0).
     */
    public void setLEDsColor(int red, int green, int blue, int white, double brightness) {
        setAllRed = red;
        setAllGreen = green;
        setAllBlue = blue;
        setAllWhite = white;
        setAllBrightness = brightness;
        m_toAnimate = null;
    } 
    
    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(m_toAnimate == null) {
            m_candle.setLEDs(setAllRed, setAllGreen, setAllBlue, setAllWhite, 0, LedCount);
            m_candle.configBrightnessScalar(setAllBrightness, 0);
        } else {
            m_candle.animate(m_toAnimate);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
