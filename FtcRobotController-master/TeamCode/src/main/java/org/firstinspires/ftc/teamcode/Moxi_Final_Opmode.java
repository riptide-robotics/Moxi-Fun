package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.ArrayList;

// USE THIS OPMODE, NOT THE OTHER ONE
// THIS IS THE FINAL MOXI OPMODE, HENCE THE NAME
// idk what else to tell you

@Config
@TeleOp(name = "Moxi Final Opmode", group = "Run This")
public class Moxi_Final_Opmode extends LinearOpMode {
    //public static int tunedRPM = 2055; 2000; 1975
    public static int tunedRPM = 2000;
    public static double kf = 0.000225;
    public static double queueSize = 25;
    public DcMotor motor;
    public Servo gate;
    private DigitalChannel button;
    // public double cooldown = -100;
    double time = 0;
    double elapsed = 0;
    Telemetry tele;
    double prevPos = 0;
    double currPos = 0;
    double downPos = 0.995;
    double upPos = 0.93;
    private PIDController rpmcontroller = new PIDController(0.0002, 0.000003, 0.00003556);
    private ArrayList<Double> records = new ArrayList<>();

    public long gateOpenDuration = 0;
    @Override
    public void runOpMode() {

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.dcMotor.get("turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate = hardwareMap.servo.get("rotator");
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(downPos);

        button = hardwareMap.get(DigitalChannel.class, "arcadeButton");
        button.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (isStopRequested()) return;

        Long startTime = null;

        while (opModeIsActive()) {
            Boolean gateUp = null;

            // button pressed
            if(!button.getState()) {
                elapsed = System.currentTimeMillis() / 1000f;
                telemetry.addData("Button State", "PRESSED! 🔵");
                startTime = System.currentTimeMillis();

            } else {
                telemetry.addData("Button State", "Not Pressed");
            }

            if (startTime != null) gateUp = ((System.currentTimeMillis() - startTime) / 7500f) < 0.3f;


            // turret times out after 20 seconds
            if(System.currentTimeMillis() / 1000f - elapsed < 22) {
                if (System.currentTimeMillis() / 1000f - elapsed > 19) {
                    gateUp = false;
                } else if (System.currentTimeMillis() / 1000f - elapsed > 2) {
                    gateUp = true;
                }

                telemetry.addData("Timeout Time: ", System.currentTimeMillis() / 1e3 - elapsed);
                runMotor();
//                if (System.currentTimeMillis() / 1e3 - elapsed < 0.3) {
//                    motor.setPower(0.2);
//                } else if (System.currentTimeMillis() / 1e3 - elapsed < 0.575) {
//                    motor.setPower(-0.2);
//                    gate.setPosition(upPos);
//                } else if (System.currentTimeMillis() / 1e3 - elapsed < 0.6 ) {
//                    motor.setPower(0);
//                } else {
//                    if (System.currentTimeMillis() / 1e3 - elapsed > 18) {
//                        gate.setPosition(downPos);
//                    } else if (System.currentTimeMillis() / 1e3 - elapsed > 1) {
//                        gate.setPosition(upPos);
//                    }
//
//                    telemetry.addData("Timeout Time: ", System.currentTimeMillis() / 1e3 - elapsed);
//                    runMotor();
//                }
            } else {
                telemetry.addLine("TIMED OUT");
                motor.setPower(0);
            }

            tele.update();
            if (gateUp != null) gate.setPosition(gateUp ? upPos : downPos);
        }
    }

    public void runMotor() {
        prevPos = currPos;
        currPos = motor.getCurrentPosition();

        tele.addData("pos", motor.getCurrentPosition());
        double dTheta = (currPos - prevPos) / 28;   //rotations?

        double dt = System.nanoTime() / 1e9 - time;
        time = System.nanoTime() / 1e9;

        double currRPM = dTheta / (dt / 60);

        //stop calculations if rpm cant be actually calculated
//        if (currRPM <= 0 && cooldown < 0.25) {
//            cooldown += dt;
//            return;
//        }
//        if (currRPM > 0) cooldown = 0;

        records.add(currRPM);
        while (records.size() > queueSize) records.remove(0);

        double undividedAverage = 0;

        for (int i = 0; i < records.size(); i++) {
            undividedAverage += records.get(i);
        }

        double average = (records.size() == queueSize) ? undividedAverage / queueSize: currRPM;

        tele.addData("average", average);
        tele.addData("CurrRPM", currRPM);
        double wantedWheelPowerAverage = rpmcontroller.calculate(average, tunedRPM) + (tunedRPM * kf);
        if (wantedWheelPowerAverage == 0) wantedWheelPowerAverage = tunedRPM;

        tele.addData("AttemptedRPM", wantedWheelPowerAverage);

        motor.setPower(wantedWheelPowerAverage);
    }
}
