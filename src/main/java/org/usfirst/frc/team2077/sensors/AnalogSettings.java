/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.sensors;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSettings {

	private final Map<Integer,AnalogInput>  analogInput_ = new HashMap<>();

	public AnalogSettings(int... analogInputNumber) {
		for (int i : analogInputNumber) {
			analogInput_.put(i, new AnalogInput(i));
		}
	}

	/** @return Analog input setting, scaled to range 0.0 - 1.0. */
	public double get(int i) {
		return analogInput_.get(i).getVoltage() / 5.;
	}
}
