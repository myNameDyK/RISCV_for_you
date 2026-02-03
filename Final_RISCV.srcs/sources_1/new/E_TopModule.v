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
    input clk,
    input rst,

    // ===== Control từ ID/EX =====
    input RegWriteE,
    input [1:0] ResultSrcE,
    input MemWriteE,
    input ALUSrcE,
    input [3:0] ALUControlE,
    input BranchE,
    input JumpE,
    input JalrE,

    // ===== Data từ ID/EX =====
    input [31:0] RD1E,
    input [31:0] RD2E,
    input [31:0] ImmExtE,
    input [31:0] PCE,
    input [31:0] PCPlus4E,
    input [4:0] RdE,
    input [2:0] funct3E,

    // ===== Forward control =====
    input [1:0] ForwardAE,
    input [1:0] ForwardBE,

    // ===== Forward data =====
    input [31:0] ALUResultM,
    input [31:0] ResultW,

    // ===== Output sang IF =====
    output PCSrcE,
    output [31:0] PCTargetE,

    // ===== Output sang MEM =====
    output RegWriteM,
    output [1:0] ResultSrcM,
    output MemWriteM,
    output [31:0] ALUResultM_out,
    output [31:0] WriteDataM,
    output [4:0] RdM,
    output [31:0] PCPlus4M
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
        .ALUSrcE     (ALUSrcE),
        .MuxSrcBE    (MuxSrcBE),
        .ImmExtE     (ImmExtE),
        .SrcBE       (SrcBE)
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
        .SrcAE       (SrcAE),
        .SrcBE       (MuxSrcBE),
        .funct3E     (funct3E),
        .BranchE     (BranchE),
        .JumpE       (JumpE),
        .PCSrcE      (PCSrcE)
    );

    // ===== PC target =====
    PC_Target_Unit pc_target (
        .PCE         (PCE),
        .SrcAE       (SrcAE),
        .ImmExtE     (ImmExtE),
        .JumpE       (JumpE),
        .JalrE       (JalrE),
        .PCTargetE   (PCTargetE)
    );

    // ===== EX/MEM register =====
    Execute_Stage_Register ex_mem (
        .clk         (clk),
        .rst         (rst),
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
    //======input signal for control =======
    input rst,
    input clk,

    input RegWriteE,
    input [1:0] ResultSrcE,
    input MemWriteE,

    // data signals in
    input [31:0] ALUResultE,
    input [31:0] WriteDataE,
    input [4:0] RdE,
    input [31:0] PCPlus4E,

    // control signals out
    output reg RegWriteM,
    output reg [1:0] ResultSrcM,
    output reg MemWriteM,

    // data signals out
    output reg [31:0] ALUResultM,
    output reg [31:0] WriteDataM,
    output reg [4:0] RdM,
    output reg [31:0] PCPlus4M
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
    input [31:0] PCE,
    input [31:0] SrcAE,
    input [31:0] ImmExtE,
    input JumpE,
    input JalrE,
    output [31:0] PCTargetE
);

    assign PCTargetE = JalrE ?
                       ((SrcAE + ImmExtE) & 32'hFFFFFFFE) :
                       (PCE + ImmExtE);

endmodule


module Branch_Unit (
    input [31:0] SrcAE,
    input [31:0] SrcBE,
    input [2:0] Funct3E,
    input BranchE,
    input JumpE,
    output PCSrcE
);

    reg take_branch;

    always @(*) begin
        take_branch = 1'b0;
        if (BranchE) begin
            case (Funct3E)
                3'b000: take_branch = (SrcAE == SrcBE);                  // BEQ
                3'b001: take_branch = (SrcAE != SrcBE);                  // BNE
                3'b100: take_branch = ($signed(SrcAE) < $signed(SrcBE)); // BLT
                3'b101: take_branch = ($signed(SrcAE) >= $signed(SrcBE));// BGE
                3'b110: take_branch = (SrcAE < SrcBE);                   // BLTU
                3'b111: take_branch = (SrcAE >= SrcBE);                  // BGEU
            endcase
        end
    end

    assign PCSrcE = JumpE | take_branch;

endmodule


module E_Mux (
    input ALUSrcE,
    input [31:0] MuxSrcBE,
    input [31:0] ImmExtE,
    output [31:0] SrcBE
);

    assign SrcBE = (ALUSrcE) ? ImmExtE : MuxSrcBE;

endmodule


module E_ForwardAE_Mux (
    input [1:0] ForwardAE,
    input [31:0] RD1E,
    input [31:0] ResultW,
    input [31:0] ALUResultM,
    output [31:0] SrcAE
);

    localparam RD        = 2'b00;
    localparam RESULTW   = 2'b01;
    localparam ALUResult = 2'b10;

    assign SrcAE =
        (ForwardAE == RD)        ? RD1E :
        (ForwardAE == RESULTW)   ? ResultW :
        (ForwardAE == ALUResult) ? ALUResultM :
                                   32'b0;

endmodule


module E_ForwardBE_Mux (
    input [1:0] ForwardBE,
    input [31:0] RD2E,
    input [31:0] ResultW,
    input [31:0] ALUResultM,
    output [31:0] MuxSrcBE
);

    localparam RD        = 2'b00;
    localparam RESULTW   = 2'b01;
    localparam ALUResult = 2'b10;

    assign MuxSrcBE =
        (ForwardBE == RD)        ? RD2E :
        (ForwardBE == RESULTW)   ? ResultW :
        (ForwardBE == ALUResult) ? ALUResultM :
                                   32'b0;

endmodule


module E_ALU (
    input [3:0] ALUControlE,
    input [31:0] SrcAE,
    input [31:0] SrcBE,
    output reg [31:0] ALUResultE
);

    localparam aluADD  = 4'b0000;
    localparam aluSUB  = 4'b0001;
    localparam aluAND  = 4'b0010;
    localparam aluOR   = 4'b0011;
    localparam aluXOR  = 4'b0100;
    localparam aluSLT  = 4'b1000;
    localparam aluSLTU = 4'b1001;
    localparam aluSLL  = 4'b0101;
    localparam aluSRL  = 4'b0110;
    localparam aluSRA  = 4'b0111;

    always @(*) begin
        ALUResultE = 32'b0;
        case (ALUControlE)
            aluADD:  ALUResultE = SrcAE + SrcBE;
            aluSUB:  ALUResultE = SrcAE - SrcBE;
            aluAND:  ALUResultE = SrcAE & SrcBE;
            aluOR :  ALUResultE = SrcAE | SrcBE;
            aluXOR:  ALUResultE = SrcAE ^ SrcBE;
            aluSLTU: ALUResultE = (SrcAE < SrcBE) ? 32'b1 : 32'b0;
            aluSLT : ALUResultE = ($signed(SrcAE) < $signed(SrcBE)) ? 32'b1 : 32'b0;
            aluSLL:  ALUResultE = SrcAE << SrcBE[4:0];
            aluSRL:  ALUResultE = SrcAE >> SrcBE[4:0];
            aluSRA:  ALUResultE = $signed(SrcAE) >>> SrcBE[4:0];
        endcase
    end

endmodule







