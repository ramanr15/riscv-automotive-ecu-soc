`timescale 1ns / 1ps
// ============================================================
// instr_mem  -- SHRUNK
//
// WHY THIS CHANGED
//   The read here is asynchronous, so block RAM cannot be
//   inferred no matter what ram_style says -- BRAM output is
//   always registered. Vivado silently fell back to
//   distributed RAM, which is why synthesis reported 0 BRAM
//   under the CPU and a large slice of the 7337 LUTs went to
//   instruction storage.
//
//   Making it real BRAM means a registered fetch, which adds a
//   pipeline stage to IF. That is a bigger change than it
//   sounds and is not worth doing right now. Shrinking the
//   array is free and cuts the distributed-RAM cost by 4x.
//
//   DEPTH_LOG2 = 8   ->  256 words, 1 KB
//   DEPTH_LOG2 = 9   ->  512 words, 2 KB  (default)
//   DEPTH_LOG2 = 10  -> 1024 words, 4 KB  (the old size)
//
//   The current test program is 78 words. 512 leaves plenty of
//   headroom, including the trap handler at 0x100 (word 64).
//   Raise it if a program ever outgrows it -- the only cost is
//   LUTs.
// ============================================================
module instr_mem #(
    parameter DEPTH_LOG2 = 9
)(
    input  wire [31:0] addr,
    output wire [31:0] instruction
);

    localparam DEPTH = (1 << DEPTH_LOG2);

    (* ram_style = "distributed" *)
    reg [31:0] memory [0:DEPTH-1];

    integer i;

    // ============================================================
    // INITIALIZATION
    // ============================================================
    initial begin
        // Fill unused locations with RISC-V NOP
        for (i = 0; i < DEPTH; i = i + 1)
            memory[i] = 32'h00000013;

        // Load machine-code program
        $readmemh("program.mem", memory);
    end

    // ============================================================
    // INSTRUCTION FETCH
    // ============================================================
    // PC is byte addressed; drop the low two bits to get a word
    // index, then keep only as many bits as the array needs.
    assign instruction = memory[addr[DEPTH_LOG2+1:2]];

endmodule
