package edu.exeter.apex.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;

    public  Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void intake(){
        intakeMotor.setPower(0.9);
    }

    public void reject(){
        intakeMotor.setPower(-0.9);
    }

    public void intakeStop(){
        intakeMotor.setPower(0);
    }


}
