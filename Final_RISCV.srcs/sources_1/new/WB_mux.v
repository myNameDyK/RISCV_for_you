`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/20/2026 10:17:48 AM
// Design Name: 
// Module Name: WB_mux
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


module WB_mux(
    input  [1:0]  ResultSrcW,
    input  [31:0] ReadDataW,
    input  [31:0] ALUResultW,
    input  [31:0] PCPlus4W,
    output [31:0] ResultW
    );
    //==========encode resultSrc=========
    localparam RESULT_ALU   = 2'b00;
    localparam RESULT_MEM   = 2'b01;
    localparam RESULT_PC4   = 2'b10;
    
    assign ResultW = (ResultSrcW == RESULT_ALU) ?   ALUResultW  :
                     (ResultSrcW == RESULT_MEM) ?   ReadDataW   :
                     (ResultSrcW == RESULT_PC4) ?   PCPlus4W    :
                                                    32'b0;
endmodule
