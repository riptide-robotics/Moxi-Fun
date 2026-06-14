package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Arcade Button Test", group = "Test")
public class ArcadeButtonTester extends LinearOpMode {

    // Declare the digital channel
    private DigitalChannel button;

    @Override
    public void runOpMode() {
        // Map the button from the configuration
        button = hardwareMap.get(DigitalChannel.class, "arcadeButton");

        // Set the mode to INPUT so it reads data
        button.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized. Awaiting Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read the button state.
            // It returns 'false' when pressed, so we invert it with '!' for intuitive logic.
            boolean isPressed = !button.getState();

            // Display the result on the Driver Station
            if (isPressed) {
                telemetry.addData("Button State", "PRESSED! 🔵");
            } else {
                telemetry.addData("Button State", "Not Pressed");
            }

            telemetry.update();
        }
    }
}
