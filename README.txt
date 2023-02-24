@Authors:
Kokogias Stefanos		
Vlachou-Konchylaki Martha 	

In order to run simulation the run_GraphSLAM.m (with parameters: mapfile, motion type, verbose) 
function must be called.

the function can be run with 2 different motion types:
# motion type = 0 linear motion along the diagonal of the map
# motion type = 1 circular motion along a circle with center (400,0) and radius = 400 pixels

the verbose parameter defines the amount of information given
# verbose = 0 only graph information is given
# verbose = 1 + information about errors in position and landmark estimation
# verbose = 2 + information about estimated position and landmarks

e.g. run_GraphSLAM('landmarks50.txt',1,1);

initial parameters (can be changed inside the initialize_2D.m function)
# poses (number of time steps)   = 2000 (for linear diagonal motion)
				 = 500 (for circular motion)
# initial robot position         = (0,0)
# map size		         = 600x600 pixels
# measurement range	         = 150 pixels

# motion noise		    = 0.2 pixels 
# measurement noise	    = 2 pixels
	(both simulated as gaussian noise)

for the linear motion
# velocity           = 2.5 pixels/timestep (on both axes)
# dt (time step)     = 0.1;

for the circular motion (parameter declared in move_circular.m)
# angle step         = 0.0034 rad (~ 0.2*1 degrees)
