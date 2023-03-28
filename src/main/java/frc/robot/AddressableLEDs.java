// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ColorConstants;

public class AddressableLEDs {
    // private static final int NUMBER_OF_LEDS = 10; // number of LED's on the Strip
    private AddressableLED led; // Creates the new object, on port 0
    private final AddressableLEDBuffer ledBuffer;

    public Hashtable<String, Object> state = new Hashtable<String, Object>();

    /** Creates a new AddressableLED. */
    public AddressableLEDs(int port, int numberOfLEDs) {
        led = new AddressableLED(port);
        led.setLength(numberOfLEDs);
        ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
        led.setData(ledBuffer);
    }

    public AddressableLED getLED() {
        return led;
    }

    public void start() {
        led.start();
    }

    public void stop() {
        led.stop();
    }

    public void setColor(Color8Bit color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void setColor(Color8Bit color, int index) {
        ledBuffer.setRGB(index, color.red, color.green, color.blue);
    }

    public void commitColor() {
        led.setData(ledBuffer);
    }

    public void left(Color8Bit start, int startPoint, int endPoint) {
        Color8Bit temp = ledBuffer.getLED8Bit(startPoint);
        for (var i = startPoint + 1; i < endPoint; i++) {
            Color8Bit color = ledBuffer.getLED8Bit(i);
            ledBuffer.setRGB(i - 1, color.red, color.green, color.blue);
        }
        ledBuffer.setRGB(endPoint - 1, temp.red, temp.green, temp.blue);

        commitColor();
    }

    public void right(Color8Bit start, int startPoint, int endPoint) {
        Color8Bit temp = ledBuffer.getLED8Bit(endPoint - 1);
        for (var i = endPoint - 2; i >= startPoint; i--) {
            Color8Bit color = ledBuffer.getLED8Bit(i);
            ledBuffer.setRGB(i + 1, color.red, color.green, color.blue);
        }
        ledBuffer.setRGB(startPoint, temp.red, temp.green, temp.blue);

        commitColor();
    }

    public void setInitTurning(Color8Bit color, int startPoint, int endPoint) {
        // create the dashes in red
        int on = 1;
        for (var i = startPoint; i < endPoint; i++) {
            if (on >= 5) {
                ledBuffer.setRGB(i, 0, 0, 0);
            } else {
                ledBuffer.setRGB(i, color.red, color.green, color.blue);
            }
            if (on == 10) {
                on = 0;
            }
            on += 1;
        }
        commitColor();

    }

    public void breathe(Color8Bit color) {
        if (state.get("state") != "breathe") {
            state.clear();
            state.put("state", "breathe");
            state.put("loop", 1);
            state.put("factor", 1);
            state.put("brightening", true);

            commitColor();
        }
        int loop = Math.max(1, (int) state.get("factor"));
        setColor(new Color8Bit((int) color.red / loop, (int) color.green / loop,
                (int) color.blue / loop));

        if ((int) state.get("loop") == 1) {
            if ((boolean) state.get("brightening")) {
                state.put("factor", (int) state.get("factor") + 1);
            } else {
                state.put("factor", (int) state.get("factor") - 1);
            }
        }

        if ((boolean) state.get("brightening") && (int) state.get("factor") >= 10) {
            state.put("brightening", false);
            state.put("factor", 9);
        } else if ((boolean) state.get("brightening") == false && (int) state.get("factor") <= 3) {
            state.put("brightening", true);
            state.put("factor", 4);
        }

        state.put("loop", (int) state.get("loop") + 1);

        if ((int) state.get("loop") == 3) {
            state.put("loop", 0);
        }

        commitColor();
    }

    public void rainbow() {
        if (state.get("state") != "rainbow") {
            state.clear();
            state.put("state", "rainbow");
            state.put("rainbowFirstPixelHue", 0);
        }

        int rainbowFirstPixelHue = (int) state.get("rainbowFirstPixelHue");
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;

        state.put("rainbowFirstPixelHue", rainbowFirstPixelHue);
        commitColor();
    }

    public void cautionBlink() {
        if (state.get("state") != "cautionBlink") {
            state.clear();
            state.put("state", "cautionBlink");
            state.put("even", 0);
        }
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            if ((int) state.get("even") < 4) {
                if (i % 2 == 0) {
                    ledBuffer.setRGB(i, 255, 0, 0);
                } else {
                    ledBuffer.setRGB(i, 0, 0, 0);
                }
            } else {
                if (i % 2 == 0) {
                    ledBuffer.setRGB(i, 0, 0, 0);
                } else {
                    ledBuffer.setRGB(i, 255, 0, 0);
                }
            }
        }

        state.put("even", (int) state.get("even") + 1);

        if ((int) state.get("even") == 8) {
            state.put("even", 0);
        }

        commitColor();
    }
}