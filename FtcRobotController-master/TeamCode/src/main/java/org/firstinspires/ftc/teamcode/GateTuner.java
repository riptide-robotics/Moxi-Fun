package org.firstinspires.ftc.teamcode;

import androidx.collection.ArraySet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.ArrayList;

@Config
@TeleOp(name = "Gate Tuner", group = "Tuner")
public class GateTuner extends LinearOpMode {

    public static double pos = 1;
    public Servo gate;
    Telemetry tele;

    @Override
    public void runOpMode() {

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gate = hardwareMap.servo.get("rotator");
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(pos);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            tele.addData("WantedPos", pos);
            tele.update();
            gate.setPosition(pos);
        }
    }
}
