package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

@TeleOp(name = "motor test")
public class MotorTest extends OpModeEX {
	private DcMotorEx fl, bl, br, fr;
	
	@Override
	public void registerSubsystems() {
	
	}
	
	@Override
	public void initEX() {
		fl = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "fl"));
		fl.setDirection(DcMotorSimple.Direction.FORWARD); // set the required motors to reverse
		bl = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "bl"));
		bl.setDirection(DcMotorSimple.Direction.FORWARD);
		br = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "br"));
		br.setDirection(DcMotorSimple.Direction.FORWARD);
		fr = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "fr"));
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
	}
	
	@Override
	public void registerBindings() {
	
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
	
	}
	
	@Override
	public void loopEX() {
		if (gamepadEX1().a().state()) {
			fl.setPower(1.0);
		} else {
			fl.setPower(0.0);
		}
		if (gamepadEX1().b().state()) {
			bl.setPower(1.0);
		} else {
			bl.setPower(0.0);
		}
		if (gamepadEX1().x().state()) {
			br.setPower(1.0);
		} else {
			br.setPower(0.0);
		}
		if (gamepadEX1().y().state()) {
			fr.setPower(1.0);
		} else {
			fr.setPower(0.0);
		}
	}
	
	@Override
	public void stopEX() {
	
	}
}
