`timescale 1ns / 1ps
// ============================================================
// fifo_sync : Reusable synchronous FIFO template
// Person 2 - Step 3
//
// Parameterised by data width and depth (depth = 2**DEPTH_LOG2).
// Pointers are one bit wider than needed to index the memory:
//   equal pointers, same MSB      -> empty
//   equal lower bits, differing MSB -> full
// This is the standard technique that avoids extra flags.
// ============================================================
module fifo_sync #(
    parameter WIDTH      = 8,
    parameter DEPTH_LOG2 = 4          // depth = 16
)(
    input  wire                    clk,
    input  wire                    reset,

    input  wire                    push,
    input  wire [WIDTH-1:0]        din,

    input  wire                    pop,
    output wire [WIDTH-1:0]        dout,

    output wire                    full,
    output wire                    empty,
    output wire [DEPTH_LOG2:0]     count
);

    localparam DEPTH = (1 << DEPTH_LOG2);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    reg [DEPTH_LOG2:0] wptr;
    reg [DEPTH_LOG2:0] rptr;

    assign empty = (wptr == rptr);

    assign full  = (wptr[DEPTH_LOG2] != rptr[DEPTH_LOG2]) &&
                   (wptr[DEPTH_LOG2-1:0] == rptr[DEPTH_LOG2-1:0]);

    assign count = wptr - rptr;

    // Asynchronous read of the head entry (distributed RAM).
    assign dout  = mem[rptr[DEPTH_LOG2-1:0]];

    always @(posedge clk) begin
        if (reset) begin
            wptr <= {(DEPTH_LOG2+1){1'b0}};
            rptr <= {(DEPTH_LOG2+1){1'b0}};
        end
        else begin
            if (push && !full) begin
                mem[wptr[DEPTH_LOG2-1:0]] <= din;
                wptr <= wptr + 1'b1;
            end
            if (pop && !empty) begin
                rptr <= rptr + 1'b1;
            end
        end
    end

endmodule
