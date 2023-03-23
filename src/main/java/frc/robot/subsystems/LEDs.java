// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.*;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddressableLEDs;

public class LEDs extends SubsystemBase {
    private AddressableLEDs leds = new AddressableLEDs(1, 60);

    /** Creates a new AdressableLEDs. */
    public LEDs() {
        leds.start();
        leds.setColor(RED);
        leds.commitColor();
    }

    public void setGamePieceColor(boolean isYellow) {
        if (isYellow) {
            leds.setColor(YELLOW);
        } else {
            leds.setColor(PURPLE);
        }
        leds.commitColor();
    }

    public void setColor(Color8Bit color, int index) {
        leds.setColor(color, index);
    }

    public void commitColor() {
        leds.commitColor();
    }

    public void fillColor(Color8Bit color) {
        leds.setColor(color);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}