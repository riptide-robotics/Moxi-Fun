package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import java.util.ArrayList;

@Config
@TeleOp(name = "Moxi Final Opmode")
public class Moxi_Final_Opmode extends LinearOpMode {
    //public static int tunedRPM = 2550;
    public static int tunedRPM = 2075;
    public static double kf = 0.000225;
    public static double queueSize = 25;
    public DcMotor motor;
    private DigitalChannel button;
    // public double cooldown = -100;
    double time = 0;
    double elapsed = 0;
    Telemetry tele;
    double prevPos = 0;
    double currPos = 0;
    private PIDController rpmcontroller = new PIDController(0.0002, 0.000003, 0.00003556);
    private ArrayList<Double> records = new ArrayList<>();

    @Override
    public void runOpMode() {

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.dcMotor.get("turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        button = hardwareMap.get(DigitalChannel.class, "arcadeButton");
        button.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // button pressed
            if(!button.getState()) {
                elapsed = System.currentTimeMillis() / 1e3;
                telemetry.addData("Button State", "PRESSED! 🔵");
            } else {
                telemetry.addData("Button State", "Not Pressed");
            }

            // turret times out after 20 seconds
            if(System.currentTimeMillis() / 1e3 - elapsed < 20.5) {
                if (System.currentTimeMillis() / 1e3 - elapsed < 0.3) {
                    motor.setPower(-0.075);
                } else if (System.currentTimeMillis() / 1e3 - elapsed < 0.45) {
                    motor.setPower(0);
                } else {
                    telemetry.addData("Timeout Time: ", System.currentTimeMillis() / 1e3 - elapsed);
                    runMotor();
                }
            } else {
                telemetry.addLine("TIMED OUT");
                motor.setPower(0);
            }

            tele.update();
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
