`timescale 1ns / 1ps

module servocount_tb;

	// Inputs
	reg clk;
	reg in0, in1;
	reg scan_in;
	reg scan_en;

	// Outputs
	wire scan_out;

	// Instantiate the Unit Under Test (UUT)
	servocount uut (
		.clk(clk), 
		.in0(in0), 
		.in1(in1), 
		.scan_out(scan_out),
		.scan_en(scan_en)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		in0 = 0;
    in1 = 0;
		scan_in = 0;
		scan_en = 0;

		#10000; // nothing for 10 us
		
		scan_en = 1; // Start reset for 14us
		#28000
		
		scan_en = 0; // Normal wait state. 50us
		#50000
		
		in0 = 1;  // Start capturing. 1835us
		#1715000
	
		in1 = 1;  // Start capturing. 240us
		#120000
	
		in0 = 0;
		#120000		

    in1 = 0; // Done. Prove no overflow... 2ms
		#2000000		

		scan_en = 1; // Rescan-out for 14us
		#28000
		
		scan_en = 0; // Normal wait state. 50us
		#50000
        
		$stop;
    $finish;
	end

  always  
    #500 clk = !clk;


endmodule

