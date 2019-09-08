`timescale 1ns / 1ps

module servocount1 (clk, in, scan_in, scan_out, scan_en);

input clk, in, scan_in, scan_en;
output scan_out;

parameter WAIT=2'b00, RUNNING=2'b01, DONE=2'b10, OVERFLOW=2'b11;

reg[1:0] inb;
reg[1:0] state;
reg[11:0] count;

wire scan_out = state[1];

// 12-bit counter with overflow 
wire[11:0] next_count;
wire overflow;
assign {overflow, next_count} = {1'b0, count} + 1;

wire raising_edge = inb[0] & ~inb[1];
wire falling_edge = ~inb[0] & inb[1];

always @(posedge clk)
begin
  if (scan_en) begin
    count <= {count[10:0], scan_in};
    state <= {state[0], count[11]};
  end
  else begin
    if (state == WAIT) begin
      if (raising_edge) begin
        state <= RUNNING;
      end
    end else if (state == RUNNING) begin
      count <= next_count;
      if (falling_edge) begin
        state <= DONE;
      end
      else if (overflow) begin
        state <= OVERFLOW;
      end
    end
  end
  inb <= {inb[0], in};
end

endmodule