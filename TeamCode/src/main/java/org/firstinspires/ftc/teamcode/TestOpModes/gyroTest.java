package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by FTC 4316 on 11/24/2017.
 */
@TeleOp(name = "Gyro Test", group = "Test")
public class gyroTest extends OpMode{
    private NavxMicroNavigationSensor gyro;

    @Override
    public void init(){
        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

    }
    @Override
    public void loop(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);

        telemetry.addLine()
                .addData("x", formatAngle(angles.angleUnit, angles.firstAngle))
                .addData("y", formatAngle(angles.angleUnit, angles.secondAngle))
                .addData("z", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/sec", formatRate(rates.zRotationRate));
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}


