package frc.robot.vision;

import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Cameras {

	private static PixyCamera pixycam = null;

	public static void setup() {
		pixycam = new PixyCamera(new SPILink());
	}

	public static PixyCamera getPixyCamera() {
		return pixycam;
	}
}