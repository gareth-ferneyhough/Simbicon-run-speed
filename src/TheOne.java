
public class TheOne {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Simbicon sim = new Simbicon();
		
		sim.init( 	Float.parseFloat(args[0]),  Float.parseFloat(args[1]),  Float.parseFloat(args[2]), 
					Float.parseFloat(args[3]),  Float.parseFloat(args[4]),  Float.parseFloat(args[5]),
					Float.parseFloat(args[6]),  Float.parseFloat(args[7]),  Float.parseFloat(args[8]),
					Float.parseFloat(args[9]),  Float.parseFloat(args[10]), Float.parseFloat(args[11]),
					Float.parseFloat(args[12]), Float.parseFloat(args[13]), Float.parseFloat(args[14]),
					Float.parseFloat(args[15]), Float.parseFloat(args[16]), Float.parseFloat(args[17]),
					Float.parseFloat(args[18])	
				);
		
		System.out.println("running");
	}

}
