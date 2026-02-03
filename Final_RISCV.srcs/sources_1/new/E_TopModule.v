`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/19/2026 11:00:08 PM
// Design Name: 
// Module Name: D_TopModule
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

//==================================================
// EXECUTE TOP MODULE
//==================================================
module Execute_TopModule (
    // ===== Clock & reset =====
    input  wire        clk,
    input  wire        rst,

    // ===== Control từ ID/EX =====
    input  wire        RegWriteE,
    input  wire [1:0]  ResultSrcE,
    input  wire        MemWriteE,
    input  wire        ALUSrcE,
    input  wire [3:0]  ALUControlE,
    input  wire        BranchE,
    input  wire        JumpE,
    input  wire        JalrE,

    // ===== Data từ ID/EX =====
    input  wire [31:0] RD1E,
    input  wire [31:0] RD2E,
    input  wire [31:0] ImmExtE,
    input  wire [31:0] PCE,
    input  wire [31:0] PCPlus4E,
    input  wire [4:0]  RdE,
    input  wire [2:0]  Funct3E,

    // ===== Forward control =====
    input  wire [1:0]  ForwardAE,
    input  wire [1:0]  ForwardBE,

    // ===== Forward data =====
    input  wire [31:0] ALUResultM,
    input  wire [31:0] ResultW,

    // ===== Output sang IF =====
    output wire        PCSrcE,
    output wire [31:0] PCTargetE,

    // ===== Output sang MEM =====
    output wire        RegWriteM,
    output wire [1:0]  ResultSrcM,
    output wire        MemWriteM,
    output wire [31:0] ALUResultM_out,
    output wire [31:0] WriteDataM,
    output wire [4:0]  RdM,
    output wire [31:0] PCPlus4M
);

    // ===== Internal wires =====
    wire [31:0] SrcAE;
    wire [31:0] MuxSrcBE;
    wire [31:0] SrcBE;
    wire [31:0] ALUResultE;

    // ===== Forward A =====
    E_ForwardAE_Mux forwardA (
        .ForwardAE   (ForwardAE),
        .RD1E        (RD1E),
        .ResultW     (ResultW),
        .ALUResultM  (ALUResultM),
        .SrcAE       (SrcAE)
    );

    // ===== Forward B =====
    E_ForwardBE_Mux forwardB (
        .ForwardBE   (ForwardBE),
        .RD2E        (RD2E),
        .ResultW     (ResultW),
        .ALUResultM  (ALUResultM),
        .MuxSrcBE    (MuxSrcBE)
    );

    // ===== ALU SrcB mux =====
    E_Mux alu_src_mux (
        .ALUSrcE   (ALUSrcE),
        .MuxSrcBE  (MuxSrcBE),
        .ImmExtE   (ImmExtE),
        .SrcBE     (SrcBE)
    );

    // ===== ALU =====
    E_ALU alu (
        .ALUControlE (ALUControlE),
        .SrcAE       (SrcAE),
        .SrcBE       (SrcBE),
        .ALUResultE  (ALUResultE)
    );

    // ===== Branch decision =====
    Branch_Unit branch_unit (
        .SrcAE    (SrcAE),
        .SrcBE    (MuxSrcBE),
        .Funct3E  (Funct3E),
        .BranchE  (BranchE),
        .JumpE    (JumpE),
        .PCSrcE   (PCSrcE)
    );

    // ===== PC target =====
    PC_Target_Unit pc_target (
        .PCE        (PCE),
        .SrcAE      (SrcAE),
        .ImmExtE    (ImmExtE),
        .JumpE      (JumpE),
        .JalrE      (JalrE),
        .PCTargetE  (PCTargetE)
    );

    // ===== EX/MEM register =====
    Execute_Stage_Register ex_mem (
        .clk          (clk),
        .rst          (rst),

        .RegWriteE   (RegWriteE),
        .ResultSrcE  (ResultSrcE),
        .MemWriteE   (MemWriteE),
        .ALUResultE  (ALUResultE),
        .WriteDataE  (MuxSrcBE),
        .RdE         (RdE),
        .PCPlus4E    (PCPlus4E),

        .RegWriteM   (RegWriteM),
        .ResultSrcM  (ResultSrcM),
        .MemWriteM   (MemWriteM),
        .ALUResultM  (ALUResultM_out),
        .WriteDataM  (WriteDataM),
        .RdM         (RdM),
        .PCPlus4M    (PCPlus4M)
    );

endmodule







module Execute_Stage_Register (
    input  wire        clk,
    input  wire        rst,

    // Control in
    input  wire        RegWriteE,
    input  wire [1:0]  ResultSrcE,
    input  wire        MemWriteE,

    // Data in
    input  wire [31:0] ALUResultE,
    input  wire [31:0] WriteDataE,
    input  wire [4:0]  RdE,
    input  wire [31:0] PCPlus4E,

    // Control out
    output reg         RegWriteM,
    output reg  [1:0]  ResultSrcM,
    output reg         MemWriteM,

    // Data out
    output reg  [31:0] ALUResultM,
    output reg  [31:0] WriteDataM,
    output reg  [4:0]  RdM,
    output reg  [31:0] PCPlus4M
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            RegWriteM  <= 1'b0;
            ResultSrcM <= 2'b00;
            MemWriteM  <= 1'b0;
            ALUResultM <= 32'b0;
            WriteDataM <= 32'b0;
            RdM        <= 5'b0;
            PCPlus4M   <= 32'b0;
        end else begin
            RegWriteM  <= RegWriteE;
            ResultSrcM <= ResultSrcE;
            MemWriteM  <= MemWriteE;
            ALUResultM <= ALUResultE;
            WriteDataM <= WriteDataE;
            RdM        <= RdE;
            PCPlus4M   <= PCPlus4E;
        end
    end

endmodule









module PC_Target_Unit (
    input  wire [31:0] PCE,
    input  wire [31:0] SrcAE,
    input  wire [31:0] ImmExtE,
    input  wire        JumpE,
    input  wire        JalrE,
    output wire [31:0] PCTargetE
);
    assign PCTargetE = JalrE ?
                       ((SrcAE + ImmExtE) & 32'hFFFFFFFE) :
                       (PCE + ImmExtE);
endmodule






