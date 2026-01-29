`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/29/2026 07:20:17 PM
// Design Name: 
// Module Name: TB_RISC_V_Control
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module TB_RISC_V_Control;


    // =====================
    // Clock & Reset
    // =====================
    reg clk;
    reg rst;

    // =====================
    // Instantiate DUT
    // =====================
    RISC_V_Control dut (
        .clk(clk),
        .rst(rst)
    );

    // =====================
    // Clock generation
    // =====================
    initial begin
        clk = 0;
        forever #5 clk = ~clk;   // 100 MHz
    end

    // =====================
    // Reset sequence
    // =====================
    initial begin
        rst = 1'b1;

        // gi? reset ?? lâu ?? pipeline clear
        #20;
        rst = 1'b0;
    end

    // =====================
    // Simulation control
    // =====================
    initial begin
        // ch? reset xong
        @(negedge rst);

        // ch?y 200 chu k?
        repeat (200) @(posedge clk);

        $display("Simulation finished.");
        $stop;
    end

    // =====================
    // Debug monitors (r?t nên có)
    // =====================
    initial begin
        $display("Time | PC | InstrD | ALUResultE | ResultW");
        $monitor("%4t | %h | %h | %h | %h",
                 $time,
                 dut.PCD,
                 dut.InstrD,
                 dut.ALUResultE,
                 dut.ResultW);
    end

endmodule
