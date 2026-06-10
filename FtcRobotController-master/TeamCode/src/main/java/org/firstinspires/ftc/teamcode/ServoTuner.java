package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "TurretPIDTuner")
public class ServoTuner extends LinearOpMode {
    public static double kp = 0.0002, ki = 0.000003, kd = 0.00003556;
    public static double lo = 20, hi = 70;
    public Servo servo;
    double time = 0;
    Telemetry tele;
    double prevPos = 0;
    double currPos = 0;
    static double goalPos = 0;
    PIDController servoController;

    @Override
    public void runOpMode() {
        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servoController = new PIDController(kp, ki, kd);
        servo = hardwareMap.servo.get("servo");

        waitForStart();
        if (isStopRequested()) return;

        servoController.setPID(kp, ki, kd);

        while (opModeIsActive()) {
            tele.addData("goalPos", goalPos);
            tele.update();
            currPos = servo.getPosition();
            if (goalPos < lo) {
                goalPos = lo;
            } else if (goalPos > hi) {
                goalPos = hi;
            }
            if (currPos != goalPos) {
                servo.setPosition(goalPos);
            }
        }
    }
}
