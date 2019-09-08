`timescale 1ns / 1ps

module servocount (clk, in0, in1, scan_out, scan_en);
  input clk, in0, in1, scan_en;
  output scan_out;

  wire scan_out_s0;

  servocount1 s0 (
		.clk(clk), 
		.in(in0), 
		.scan_in(1'b0),
		.scan_out(scan_out_s0),
		.scan_en(scan_en)
  );

  servocount1 s1 (
		.clk(clk), 
		.in(in1), 
		.scan_in(scan_out_s0),
		.scan_out(scan_out),
		.scan_en(scan_en)
  );

endmodule