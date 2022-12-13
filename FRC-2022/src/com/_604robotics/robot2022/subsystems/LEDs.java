package com._604robotics.robot2022.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(8);
    private double startTime;
    private Trigger isLaunchMode;
    private IntSupplier redCount;
    private IntSupplier blueCount;
    private BooleanSupplier atSpeed;
    
    public LEDs(Trigger isLaunchMode, IntSupplier redCount, IntSupplier blueCount, BooleanSupplier atSpeed) {
        leds.setLength(ledBuffer.getLength());
        off();  // Start with everything off
        leds.setData(ledBuffer);
        leds.start();

        startTime = Timer.getFPGATimestamp();

        this.isLaunchMode = isLaunchMode;
        this.redCount = redCount;
        this.blueCount = blueCount;
        this.atSpeed = atSpeed;
    }

    private boolean isFirstHalfPeriod() {
        double kPeriod = 0.5;  // seconds
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        return (timeElapsed % kPeriod) < kPeriod * 0.5;
    }

    private void off() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    private void red(boolean flash) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, (flash && isFirstHalfPeriod()) ? 0 : 255, 0, 0);
        }
    }

    private void blue(boolean flash) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, (flash && isFirstHalfPeriod()) ? 0 : 255);
        }
    }

    private void flashRedBlue() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, isFirstHalfPeriod() ? 255 : 0, 0, isFirstHalfPeriod() ? 0 : 255);
        }
    }

    private void green(boolean flash) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, (flash && isFirstHalfPeriod()) ? 0 : 255, 0);
        }
    }

    @Override
    public void periodic() {
        if (isLaunchMode.get()) {
            if (atSpeed.getAsBoolean()) {
                green(false);
            } else {
                green(true);
            }
        } else {
            if (redCount.getAsInt() == 2) {
                red(false);
            } else if (blueCount.getAsInt() == 2) {
                blue(false);
            } else if (redCount.getAsInt() == 1 && blueCount.getAsInt() == 1) {
                flashRedBlue();
            } else if (redCount.getAsInt() == 1) {
                red(true);
            } else if (blueCount.getAsInt() == 1) {
                blue(true);
            } else {
                off();
            }
        }
        leds.setData(ledBuffer);
    }
}
