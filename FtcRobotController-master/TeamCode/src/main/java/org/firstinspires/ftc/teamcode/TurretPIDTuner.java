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
@TeleOp(name = "TurretPIDTuner")
public class TurretPIDTuner extends LinearOpMode {

    public static double kp = 0.00025, ki = 0.0075, kd = 0.00000155;
    public static int wantedrpm = 0;
    public static double kf = 0.0002;
    public static double queueSize = 5;
    public double lKp = kp, lKi = ki, lKd = kd;
    public DcMotor motor;
    public double cooldown = -100;
    public boolean debounce = true;
    double time = 0;
    Telemetry tele;
    double prevPos = 0;
    double currPos = 0;
    private PIDController rpmcontroller;
    private ArrayList<Double> records = new ArrayList<>();

    @Override
    public void runOpMode() {

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rpmcontroller = new PIDController(kp, ki, kd);

        motor = hardwareMap.dcMotor.get("turret");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            tele.addData("WantedRPM", wantedrpm);
            tele.update();

            if (kp != lKp || ki != lKi || kd != lKd) {
                rpmcontroller.setPID(kp, ki, kd);

                lKp = kp;
                lKi = ki;
                lKd = kd;
            }

            pidtunedmotor(wantedrpm);
        }
    }

    public void pidtunedmotor(double rpm) {
        prevPos = currPos;
        currPos = motor.getCurrentPosition();

        tele.addData("pos", motor.getCurrentPosition());
        double dTheta = (currPos - prevPos) / 28;   //rotations?

        double dt = System.nanoTime() / 1e9 - time;
        time = System.nanoTime() / 1e9;

        double currRPM = dTheta / (dt / 60);

        //stop calculations if rpm cant be actually calculated
        if (currRPM <= 0 && cooldown < 0.25) {
            cooldown += dt;
            return;
        }
        if (currRPM > 0) cooldown = 0;

        records.add(currRPM);
        while (records.size() > queueSize) records.remove(0);

        double undividedAverage = 0;

        for (int i = 0; i < records.size(); i++) {
            undividedAverage += records.get(i);
        }

        double average;

        if (records.size() == queueSize) average = undividedAverage / queueSize;
        else average = currRPM;

        tele.addData("average", average);
        tele.addData("CurrRPM", currRPM);
        double wantedWheelPowerAverage = rpmcontroller.calculate(average, rpm) + (wantedrpm * kf);
        if (wantedWheelPowerAverage == 0) wantedWheelPowerAverage = wantedrpm;

        tele.addData("AttemptedRPM", wantedWheelPowerAverage);

        motor.setPower(wantedrpm == 0 ? 0 : wantedWheelPowerAverage);
    }
}
