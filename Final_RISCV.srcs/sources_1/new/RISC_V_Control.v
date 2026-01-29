`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2026 08:17:22 PM
// Design Name: 
// Module Name: RISC_V_Control
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


module RISC_V_Control(
    input  wire clk,
    input  wire rst
);


    // FETCH -> DECODE
    wire [31:0] InstrD, PCD, PCPlus4D;


    // DECODE -> EXECUTE
    wire RegWriteE, MemWriteE, BranchE, JalrE, JumpE, ALUSrcE;
    wire [3:0] ALUControlE;
    wire [1:0] ResultSrcE;
    wire [31:0] RD1E, RD2E, ImmExtE;
    wire [4:0] RdE, Rs1E, Rs2E;
    wire [31:0] PCE, PCPlus4E;
    wire [2:0] Funct3E;
    wire [31:0] ALUResultE;


    // EXECUTE -> MEMORY
    wire RegWriteM, MemWriteM;
    wire [1:0] ResultSrcM;
    wire [31:0] ALUResultM, WriteDataM, PCPlus4M;
    wire [4:0] RdM;


    // MEMORY -> WRITEBACK
    wire RegWriteW;
    wire [1:0] ResultSrcW;
    wire [31:0] ALUResultW, ReadDataW, PCPlus4W;
    wire [4:0] RdW;
    wire [31:0] ResultW;


    // CONTROL WIRES
    wire PCSrcE;
    wire [31:0] PCTargetE;

    wire StallF, StallD, FlushD, FlushE;
    wire [1:0] ForwardAE, ForwardBE;

    wire [31:0] SrcAE, SrcBE, MuxSrcBE;
    wire [4:0] Rs1D;
    wire [4:0] Rs2D;

    // FETCH STAGE
    F_TopModule F (
        .clk(clk),
        .rst(rst),
        .stallF(StallF),
        .flushD(FlushD),
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),
        .InstrD(InstrD),
        .PCD(PCD),
        .PCPlus4D(PCPlus4D)
    );


    // DECODE STAGE
    D_TopModule D (
        .clk(clk),
        .rst(rst),
        .stallD(StallD),
        .flushE(FlushE),
        .InstrD(InstrD),
        .PCD(PCD),
        .PCPlus4D(PCPlus4D),
        .RegWriteW(RegWriteW),
        .RdW(RdW),
        .ResultW(ResultW),
        .RegWriteE(RegWriteE),
        .MemWriteE(MemWriteE),
        .BranchE(BranchE),
        .JalrE(JalrE),
        .JumpE(JumpE),
        .ALUSrcE(ALUSrcE),
        .ALUControlE(ALUControlE),
        .ResultSrcE(ResultSrcE),
        .RD1E(RD1E),
        .RD2E(RD2E),
        .ImmExtE(ImmExtE),
        .RdE(RdE),
        .PCE(PCE),
        .PCPlus4E(PCPlus4E),
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .Funct3E(Funct3E),
        .Rs1D_out(Rs1D),
        .Rs2D_out(Rs2D)
    );


    // FORWARD UNIT
    Forward_Unit FU (
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .RdM(RdM),
        .RdW(RdW),
        .RegWriteM(RegWriteM),
        .RegWriteW(RegWriteW),
        .ForwardAE(ForwardAE),
        .ResultSrcM(ResultSrcM),
        .ForwardBE(ForwardBE)
    );

    
    E_ForwardAE_Mux FA (
    .ForwardAE(ForwardAE),
    .RD1E(RD1E),
    .ResultW(ResultW),
    .ALUResultM(ALUResultM),
    .SrcAE(SrcAE)
    );


    // EXECUTE STAGE
    E_ForwardBE_Mux FB (
        .ForwardBE(ForwardBE),
        .RD2E(RD2E),
        .ResultW(ResultW),
        .ALUResultM(ALUResultM),
        .MuxSrcBE(MuxSrcBE)
    );

    E_Mux SrcBMux (
        .ALUSrcE(ALUSrcE),
        .MuxSrcBE(MuxSrcBE),
        .ImmExtE(ImmExtE),
        .SrcBE(SrcBE)
    );

    E_ALU ALU (
        .ALUControlE(ALUControlE),
        .SrcAE(SrcAE),
        .SrcBE(SrcBE),
        .ALUResultE(ALUResultE)
    );


    Branch_Unit BU (
        .SrcAE(SrcAE),
        .SrcBE(MuxSrcBE),
        .Funct3E(Funct3E),
        .BranchE(BranchE),
        .JumpE(JumpE),
        .PCSrcE(PCSrcE)
    );

    PC_Target_Unit PTU (
        .PCE(PCE),
        .SrcAE(SrcAE),
        .ImmExtE(ImmExtE),
        .JumpE(JumpE),
        .JalrE(JalrE),
        .PCTargetE(PCTargetE)
    );

    Execute_Stage_Register EX_MEM (
        .clk(clk),
        .rst(rst),
        .RegWriteE(RegWriteE),
        .ResultSrcE(ResultSrcE),
        .MemWriteE(MemWriteE),
        .ALUResultE(ALUResultE),   
        .WriteDataE(MuxSrcBE),
        .RdE(RdE),
        .PCPlus4E(PCPlus4E),
        .RegWriteM(RegWriteM),
        .ResultSrcM(ResultSrcM),
        .MemWriteM(MemWriteM),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .RdM(RdM),
        .PCPlus4M(PCPlus4M)
    );



    // MEMORY STAGE
    M_TopModule M (
        .clk(clk),
        .rst(rst),
        .ResultSrcM(ResultSrcM),
        .RegWriteM(RegWriteM),
        .MemWriteM(MemWriteM),
        .ALUResultM(ALUResultM),
        .WriteDataM(WriteDataM),
        .RdM(RdM),
        .PCPlus4M(PCPlus4M),
        .ResultSrcW(ResultSrcW),
        .RegWriteW(RegWriteW),
        .ALUResultW(ALUResultW),
        .ReadDataW(ReadDataW),
        .RdW(RdW),
        .PCPlus4W(PCPlus4W)
    );


    // WRITEBACK
    WB_mux WB (
        .ResultSrcW(ResultSrcW),
        .ReadDataW(ReadDataW),
        .ALUResultW(ALUResultW),
        .PCPlus4W(PCPlus4W),
        .ResultW(ResultW)
    );



    // HAZARD UNIT
    Hazard_Unit HU (
        .ResultSrcE(ResultSrcE),
        .Rs1D(Rs1D),
        .Rs2D(Rs2D),
        .RdE(RdE),
        .PCSrcE(PCSrcE),
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE)
    );

endmodule





//========if Ex need value in a register, however, in MEM or WB need to write a new value into this register,
//======= ALU will be forwarded the new value to use instead load old value at register
// prioprity Mem > WB because data in wb will be replaced by Mem
module Forward_Unit(
    input [4:0] Rs1E, Rs2E,
    input [4:0] RdM, RdW,
    input       RegWriteM, RegWriteW,
    input [1:0] ResultSrcM,

    output reg [1:0] ForwardAE, ForwardBE
);
    localparam RD        = 2'b00;
    localparam RESULTW   = 2'b01;
    localparam ALURESULT = 2'b10;
    localparam RESULT_MEM= 2'b01; // load

    always @(*) begin
        ForwardAE = RD;
        ForwardBE = RD;

        if (RegWriteM && (RdM != 0) && (RdM == Rs1E)
            && (ResultSrcM != RESULT_MEM))
            ForwardAE = ALURESULT;
        else if (RegWriteW && (RdW != 0) && (RdW == Rs1E))
            ForwardAE = RESULTW;

        if (RegWriteM && (RdM != 0) && (RdM == Rs2E)
            && (ResultSrcM != RESULT_MEM))
            ForwardBE = ALURESULT;
        else if (RegWriteW && (RdW != 0) && (RdW == Rs2E))
            ForwardBE = RESULTW;
    end
endmodule



//===Hazard  unit include 2 module: loaduse + control hazard unit======
//load-use data: neu nhu previous ins can read dmem va luu vao rd, va current ins use this rd,
//thi xay ra load use data, can stall 1 cycle va insert lenh nop vao exe stage

///control hazard : neu co lenh beq or jump thi add nhay se dc tinh o exe stage, 
//but da co 2 lenh khac di vao fetch va decode =>> fetch sai next ins => can remove 2 lenh do = flush (NOP)

module Hazard_Unit (
    input  wire [4:0] Rs1D,
    input  wire [4:0] Rs2D,
    input  wire [4:0] RdE,
    input  wire [1:0] ResultSrcE,   // 01 = load
    input  wire       PCSrcE,

    output wire StallF,
    output wire StallD,
    output wire FlushD,
    output wire FlushE
);

    wire loadUseHazard;

    assign loadUseHazard =  (ResultSrcE == 2'b01) &&     // load in EX
                            (RdE != 5'b0) &&
                            ( (RdE == Rs1D) || (RdE == Rs2D) );

    // Stall PC & IF/ID when load-use
    assign StallF = loadUseHazard;
    assign StallD = loadUseHazard;

    // Insert bubble into EX only for load-use
    assign FlushE = loadUseHazard | PCSrcE;

    // Flush IF/ID when branch/jump taken
    assign FlushD = PCSrcE;

endmodule





