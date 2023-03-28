// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddressableLEDs;

public class LEDs extends SubsystemBase {
    private AddressableLEDs leds = new AddressableLEDs(0, 196);
    private int[] segments = { 0, 49, 98, 147, 196 };
    private final SendableChooser<Color8Bit> color_selector = new SendableChooser<>();
    private String dir = "";
    private String pattern = "pattern";

    /** Creates a new AdressableLEDs. */
    public LEDs() {
        leds.start();
        leds.setColor(RED);
        leds.commitColor();

        color_selector.addOption("BLACK", BLACK);
        color_selector.addOption("PURPLE", PURPLE);
        color_selector.addOption("ORANGE", ORANGE);
        color_selector.addOption("YELLOW", YELLOW);
        color_selector.addOption("GREEN", GREEN);
        color_selector.addOption("BLUE", BLUE);
        color_selector.setDefaultOption("RED", RED);
        SmartDashboard.putData("LEDS", color_selector);
        SmartDashboard.putData("Reset LEDs", new InstantCommand(() -> pattern = "pattern"));
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

    public void setDir(String dir) {
        this.dir = dir;
    }

    public void setPattern(String pattern) {
        this.pattern = pattern;
    }

    @Override
    public void periodic() {
        Color8Bit selected = color_selector.getSelected();
        if (pattern == "pattern") {
            if (dir == "left") {
                if (leds.state.get("state") != "left") {
                    leds.state.put("state", "left");
                    leds.setInitTurning(selected, 0, 95);
                }
                leds.left(selected, segments[0], segments[1]);
                leds.left(selected, segments[1], segments[2]);
                leds.right(selected, segments[2], segments[3]);
                leds.right(selected, segments[3], segments[4]);
            } else if (dir == "right") {
                if (leds.state.get("state") != "right") {
                    leds.state.put("state", "right");
                    leds.setInitTurning(selected, 0, 95);
                }
                leds.right(selected, segments[0], segments[1]);
                leds.right(selected, segments[1], segments[2]);
                leds.left(selected, segments[2], segments[3]);
                leds.left(selected, segments[3], segments[4]);
            } else {
                leds.breathe(selected);
            }
        } else if (pattern == "rainbow") {
            leds.rainbow();
        } else if (pattern == "balance") {
            leds.cautionBlink();
        }
    }
}