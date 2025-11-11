package edu.exeter.apex.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class intake {
    private DcMotor intake = null;

    public void intake(){
        intake.setPower(0.9);
    }

    public void reject(){
        intake.setPower(-0.9);
    }

    public void intakeStop(){
        intake.setPower(0);
    }


}
