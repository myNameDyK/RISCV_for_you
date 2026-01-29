`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/19/2026 11:12:05 PM
// Design Name: 
// Module Name: M_TopModule
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


module M_TopModule(
    input clk,
    input rst,

    // ===== Control signals from Execute stage =====
    input [1:0]  ResultSrcM,
    input        RegWriteM,
    input        MemWriteM,          // we cho DMEM

    // ===== Data from Execute stage =====
    input [31:0] ALUResultM,
    input [31:0] WriteDataM,         // du lieu store
    input [4:0]  RdM,
    input [31:0] PCPlus4M,

    // ===== Outputs to Writeback stage =====
    output [1:0]  ResultSrcW,
    output        RegWriteW,
    output [31:0] ALUResultW,
    output [31:0] ReadDataW,
    output [4:0]  RdW,
    output [31:0] PCPlus4W
    
);

    // ===== Internal wire =====
    wire [31:0] ReadDataM;

    // ===== Data Memory =====
    M_DMEM Dmem (
        .clk(clk),
        .We(MemWriteM),
        .WD(WriteDataM),
        .ALUResultM(ALUResultM),
        .RD(ReadDataM)
    );

    // ===== Memory → Writeback Pipeline Register =====
    Memory_Stage_Register MW_Reg (
        .clk(clk),
        .rst(rst),

        // control
        .ResultSrcM(ResultSrcM),
        .RegWriteM(RegWriteM),

        // data
        .ALUResultM(ALUResultM),
        .RD(ReadDataM),
        .RdM(RdM),
        .PCPlus4M(PCPlus4M),

        // output
        .ResultSrcW(ResultSrcW),
        .RegWriteW(RegWriteW),
        .ALUResultW(ALUResultW),
        .ReadDataW(ReadDataW),
        .RdW(RdW),
        .PCPlus4W(PCPlus4W)
    );
endmodule














module M_DMEM(
    input clk,
    input We,
    input  [31:0] WD,
    input  [31:0] ALUResultM,
    output reg [31:0] RD
);
    (* ram_style = "block" *)
    reg [31:0] DMEM [1023:0];

    always @(posedge clk) begin
        if (We) begin 
            DMEM[ALUResultM[9:2]] <= WD;
        end
        RD <= DMEM[ALUResultM[9:2]];
    end   
endmodule









module Memory_Stage_Register(
    input clk,
    input rst,
    
    //===== INPUT SIGNAL =====
    input [1:0] ResultSrcM,
    input       RegWriteM,
    
    //===== INPUT DATA =====
    input [31:0] ALUResultM,
    input [31:0] RD,
    input [4:0]  RdM,
    input [31:0] PCPlus4M,
    
    //===== OUTPUT SIGNAL =====
    output reg [1:0] ResultSrcW,
    output reg       RegWriteW,
    
    //===== OUTPUT DATA =====
    output reg [31:0] ALUResultW,
    output reg [31:0] ReadDataW,
    output reg [4:0]  RdW,
    output reg [31:0] PCPlus4W
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ResultSrcW <= 2'b0;
            RegWriteW  <= 1'b0;
            ALUResultW <= 32'b0;
            ReadDataW  <= 32'b0;
            RdW        <= 5'b0;
            PCPlus4W   <= 32'b0;
        end
        else begin
            ResultSrcW <= ResultSrcM;
            RegWriteW  <= RegWriteM;
            ALUResultW <= ALUResultM;
            ReadDataW  <= RD;
            RdW        <= RdM;
            PCPlus4W   <= PCPlus4M;
        end
    end
endmodule

