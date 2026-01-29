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


module D_TopModule(
    input clk, rst, stallD, flushE,

    // ===== INPUT FROM TESTBENCH =====
    input [31:0] InstrD,
    input [31:0] PCD,
    input [31:0] PCPlus4D,
    input RegWriteW,
    input [4:0] RdW,
    input [31:0] ResultW,
    

    // ===== OUTPUT TO WAVEFORM =====
    output RegWriteE,
    output MemWriteE,
    output BranchE,
    output JalrE,
    output JumpE,
    output ALUSrcE,
    output [3:0] ALUControlE,
    output [1:0] ResultSrcE,

    output [31:0] RD1E,
    output [31:0] RD2E,
    output [31:0] ImmExtE,
    output [4:0]  RdE,
    output [31:0] PCE,
    output [31:0] PCPlus4E,
    output [2:0] Funct3E,
    output [4:0] Rs1D_out,
    output [4:0] Rs2D_out,

    //=======output for hazard detection unit=========
    output [4:0] Rs1E, Rs2E
      
);

    // ===============================
    // 1. TACH FIELD INSTRUCTION
    // ===============================
    wire [6:0] Opcode = InstrD[6:0];
    wire [4:0] Rd     = InstrD[11:7];
    wire [2:0] Funct3D = InstrD[14:12];
    wire [4:0] Rs1D    = InstrD[19:15];
    wire [4:0] Rs2D    = InstrD[24:20];
    wire [6:0] Funct7D = InstrD[31:25];


    // ===============================
    // 2. WIRE trong STAGE D
    // ===============================
    wire RegWriteD;
    wire [1:0] ResultSrcD;
    wire MemWriteD;
    wire JumpD, BranchD, JalrD;
    wire [3:0] ALUControlD;
    wire ALUSrcD;
    wire [2:0] ImmSrcD;

    wire [31:0] RD1, RD2;
    wire [31:0] ImmExtD;


    assign Rs1D_out = Rs1D;
    assign Rs2D_out = Rs2D;

    // ===============================
    // 3. CONTROL UNIT
    // ===============================
    D_controlUnit CU (
        .Opcode(Opcode),
        .Funct7D(Funct7D),
        .Funct3D(Funct3D),

        .RegWriteD(RegWriteD),
        .ResultSrcD(ResultSrcD),
        .MemWriteD(MemWriteD),
        .JumpD(JumpD),
        .BranchD(BranchD),
        .JalrD(JalrD),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        .ImmSrcD(ImmSrcD)
    );

    // ===============================
    // 4. REGISTER FILE
    // (decode test ? KH?NG write back)
    // ===============================
    D_registerFile RF (
        .clk(clk),
        .rst(rst),
        .We3(RegWriteW),
        .Rs1D(Rs1D),
        .Rs2D(Rs2D),
        .RdW(RdW),
        .Wd3(ResultW),
        .Rd1(RD1),
        .Rd2(RD2)
    );

    // ===============================
    // 5. IMMEDIATE EXTENDER
    // ===============================
    D_Extender EXT (
        .ImmSrcD(ImmSrcD),
        .InstrD(InstrD),
        .ImmExtD(ImmExtD)
    );

    // ===============================
    // 6. ID / EX PIPELINE REGISTER
    // ===============================
    Decode_Stage_Register ID_EX (
        .clk(clk),
        .rst(rst),
        .flushE(flushE),
        .stallD(stallD),

        .RegWriteD(RegWriteD),
        .ResultSrcD(ResultSrcD),
        .MemWriteD(MemWriteD),
        .JumpD(JumpD),
        .BranchD(BranchD),
        .JalrD(JalrD),
        .ALUControlD(ALUControlD),
        .ALUSrcD(ALUSrcD),
        
        .PCE(PCE),
        .PCPlus4E(PCPlus4E),
        .RD1(RD1),
        .RD2(RD2),
        .PCD(PCD),
        .PCPlus4D(PCPlus4D),
        .RdD(Rd),
        .ImmExtD(ImmExtD),
        .Funct3D(Funct3D),
        
        .RegWriteE(RegWriteE),
        .ResultSrcE(ResultSrcE),
        .MemWriteE(MemWriteE),
        .JumpE(JumpE),
        .BranchE(BranchE),
        .JalrE(JalrE),
        .ALUControlE(ALUControlE),
        .ALUSrcE(ALUSrcE),

        .RD1E(RD1E),
        .RD2E(RD2E),
        .ImmExtE(ImmExtE),
        .RdE(RdE),
        
     //======= for hazard detection unit========= 
        .Rs1E(Rs1E),
        .Rs2E(Rs2E),
        .Rs1D(Rs1D),
        .Rs2D(Rs2D)
    );

endmodule



module Decode_Stage_Register(
    input clk, rst, stallD, flushE,
    //=====INPUT SIGNAL======
    input RegWriteD,
    input [1:0] ResultSrcD,
    input MemWriteD,
    input JumpD,
    input BranchD,
    input JalrD,
    input [3:0] ALUControlD,
    input ALUSrcD,
    //======INPUT DATA=======
    input [31:0] RD1, RD2,
    input [31:0] PCD, PCPlus4D,
    input [4:0] RdD,
    input [31:0] ImmExtD,
    input  [2:0] Funct3D,
    
    //=======input for hazard detection unit=========
    input [4:0] Rs1D, Rs2D,
    
    //=====OUTPUT SIGNAL======
    output reg RegWriteE,
    output reg [1:0] ResultSrcE,
    output reg MemWriteE,
    output reg JumpE,
    output reg BranchE,
    output reg JalrE,
    output reg [3:0] ALUControlE,
    output reg ALUSrcE,
//======OUTPUT DATA=======  
    output reg [31:0] RD1E, RD2E,
    output reg [31:0] PCE, PCPlus4E,
    output reg [4:0] RdE,
    output reg [31:0] ImmExtE,
    output reg [2:0] Funct3E,
    
   //=======output for hazard detection unit=========
    output reg [4:0] Rs1E, Rs2E
     
    
    );

always @(posedge clk or posedge rst) begin
    // ===== RESET =====
    if (rst || flushE) begin
        RegWriteE   <= 1'b0;
        MemWriteE   <= 1'b0;
        BranchE     <= 1'b0;
        JalrE       <= 1'b0;
        JumpE       <= 1'b0;
        ALUSrcE     <= 1'b0;
        ALUControlE <= 4'b0000;
        ResultSrcE  <= 2'b00;
        RD1E        <= 32'b0;
        RD2E        <= 32'b0;
        ImmExtE     <= 32'b0;
        RdE         <= 5'b0;
        PCE         <= 32'b0;
        PCPlus4E    <= 32'b0;
        Rs1E        <= 5'b0;
        Rs2E        <= 5'b0;
        Funct3E     <= 3'b0;
     end
     else if(!stallD) begin
        RegWriteE   <= RegWriteD;
        MemWriteE   <= MemWriteD;
        BranchE     <= BranchD;
        JalrE       <= JalrD;
        JumpE       <= JumpD;
        ALUSrcE     <= ALUSrcD;
        ALUControlE <= ALUControlD;
        ResultSrcE  <= ResultSrcD;
    
        RD1E        <= RD1;
        RD2E        <= RD2;
        ImmExtE     <= ImmExtD;
        RdE         <= RdD;
        PCE         <= PCD;
        PCPlus4E    <= PCPlus4D;   
        Rs1E        <= Rs1D;
        Rs2E        <= Rs2D;  
        Funct3E     <= Funct3D;
     end
end        
endmodule



module D_controlUnit(
    //INPUT
    input [6:0] Opcode,
    input [6:0] Funct7D,
    input [2:0] Funct3D,
    //OUTPUT
    output              RegWriteD,
    output reg [1:0]    ResultSrcD,
    output              MemWriteD,
    output              JumpD,
    output              BranchD,
    output reg [3:0]    ALUControlD,
    output              ALUSrcD,
    output reg [2:0]    ImmSrcD,
    output JalrD
    );
    
    
// ==========regWriteD==========co ghi vao rd ko
    assign RegWriteD =  (Opcode == 7'b0110011) ||   //R
                        (Opcode == 7'b0010011) ||   // I
                        (Opcode == 7'b0000011) ||   // Load
                        (Opcode == 7'b1101111) ||   //JAL
                        (Opcode == 7'b1100111) ||   //JALR
                        (Opcode == 7'b0110111) ||   //LUI
                        (Opcode == 7'b0010111);     //AUIPC
    
    
    
// ==========resultSrcD==========Mux chon nguon ghi rd : ALU, Memory, PC+4
// ===== Result Source encoding =====
    localparam RESULT_ALU   = 2'b00;
    localparam RESULT_MEM   = 2'b01;
    localparam RESULT_PC4   = 2'b10;
    
    always @(*) begin
        case (Opcode)
            7'b0110011, // R-type
            7'b0010011, // I-type (addi, andi, ...)
            7'b0110111, // LUI
            7'b0010111: // AUIPC
                ResultSrcD = RESULT_ALU;
    
            7'b0000011: // LW
                ResultSrcD = RESULT_MEM;
    
            7'b1101111, // JAL
            7'b1100111: // JALR
                ResultSrcD = RESULT_PC4;
    
            default:
                ResultSrcD = RESULT_ALU;
        endcase
    end
    
    
//==========memWriteD==========co luu vao MEM ko
    assign MemWriteD    = (Opcode == 7'b0100011); // S-type : lenh sw
    
    
//==========jumpD==========nhay ko dieu kien 
    assign  JumpD       = (Opcode == 7'b1101111) ||   //JAL
                          (Opcode == 7'b1100111);     //JALR
    
    
//==========branchD==========nhay co dieu kien (ALU xet dieu kien)
    assign BranchD      = (Opcode == 7'b1100011);    //B-type
    

//==========JalrD : rs1 +imm==========
    assign JalrD = (Opcode == 7'b1100111); // JALR
    
    
//==========ALUControl==========ALU se lam gi : add, and, or, xor, shift........
 
    //localparam cho cac gia tri ma ALUControl co the nhan
    //=====signal for ALU======
    localparam aluADD  = 4'b0000;
    localparam aluSUB  = 4'b0001;
    localparam aluAND  = 4'b0010;
    localparam aluOR   = 4'b0011;
    localparam aluXOR  = 4'b0100;
    //=====signal for ALU Set Less Than=====
    localparam aluSLT  = 4'b1000;  // signed
    localparam aluSLTU = 4'b1001;  // unsigned
    //======signal for Barrel Shifter======
    localparam aluSLL  = 4'b0101;
    localparam aluSRL  = 4'b0110;
    localparam aluSRA  = 4'b0111;

    
    always @(*) begin
        case (Opcode)    
        //======= R-type ========           
            7'b0110011:
                begin       //xet funct3 cua R-type     
                    case (Funct3D)           
                        3'b000: // nhom add (add, sub)
                            ALUControlD = (Funct7D == 7'b0000000) ? aluADD  //0000000 lenh add
                                                                 : aluSUB;  //0100000 lenh sub
                        3'b111: // nhom AND
                            ALUControlD = aluAND;
                        3'b110: // nhom or
                            ALUControlD = aluOR;
                        3'b100: // nhom xor
                            ALUControlD = aluXOR;
                        3'b001: // nhom ssl
                            ALUControlD = aluSLL;
                        3'b101: // nhom shift (SRL, SRA)
                            ALUControlD = (Funct7D == 7'b0000000) ? aluSRL  //0000000 lenh SRL
                                                                 : aluSRA;  //0100000 lenh SRA
                                //nhom set less than (signed)                                 
                        3'b010: ALUControlD = aluSLT;
                                //nhom set less than (unsigned) 
                        3'b011: ALUControlD = aluSLTU;  
                                                               
                        default: ALUControlD = aluADD;                                                           
                    endcase                 
                 end     
        // =========== end R-type=================            
             
        //======= I-type ========     
            7'b0010011:
                 begin       //xet funct3 cua I-type     
                    case (Funct3D)           
                        3'b000: // nhom addI
                            ALUControlD = aluADD; //add
                        3'b111: // nhom ANDI
                            ALUControlD = aluAND;
                        3'b110: // nhom orI
                            ALUControlD = aluOR;
                        3'b100: // nhom xorI
                            ALUControlD = aluXOR;
                        3'b001: // nhom sllI
                            ALUControlD = aluSLL;
                                                        //nhom set less than (signed)                                 
                        3'b010: ALUControlD = aluSLT;
                                //nhom set less than (unsigned) 
                        3'b011: ALUControlD = aluSLTU; 
                        
                        3'b101: // nhom shift (SRLI, SRAI)
                            ALUControlD = (Funct7D == 7'b0000000) ? aluSRL  //0000000 lenh SRLI
                                                                 : aluSRA;  //0100000 lenh SRAI                 
                    endcase                 
                 end           
                      
        // =========== end I-type=================

           7'b0000011: ALUControlD = aluADD;  // LOAD - lenh lw (rd = Mem[ rs1 + imm ]))
           7'b0100011: ALUControlD = aluADD;  // STORE - lenh sw   (Mem[ rs1 + imm ] = rs2)
           7'b1100011: ALUControlD = aluSUB;  // BRANCH - lenh beq, bne (rs1-rs2)
           7'b1100111: ALUControlD = aluADD;  // JALR
           7'b1101111: ALUControlD = aluADD;  // JAL
           7'b0010111: ALUControlD = aluADD;  // AUIPC
           
        endcase        //endcase cua opcode
    end
 
 
 
//==========ALUSrcD==========0: dung rs2, 1: dung imm
    assign ALUSrcD =    (Opcode == 7'b0100011) ||   // Store
                        (Opcode == 7'b0010011) ||   // I
                        (Opcode == 7'b0000011) ||   // Load
                        (Opcode == 7'b1101111) ||   //JAL
                        (Opcode == 7'b1100111) ||   //JALR
                        (Opcode == 7'b0110111) ||   //LUI
                        (Opcode == 7'b0010111);     //AUIPC 
                        
                        
                        
 //==========ImmSrcD ==========cho biet imm thuoc nhom lenh gi I,S,B,U,J
    //localparam cho cac gia tri ma ImmSrcD co the nhan
    localparam immI = 3'b000;
    localparam immS = 3'b001;
    localparam immB = 3'b010;
    localparam immU = 3'b011;
    localparam immJ = 3'b100;   
    
    always @(*) begin
        case(Opcode)
            7'b0010011,    //ALU-imm
            7'b0000011,    //Load
            7'b1100111: ImmSrcD = immI;    //JALR
                     
            7'b0100011: ImmSrcD = immS;    //Store
            7'b1100011: ImmSrcD = immB;    //Branch
            7'b1101111: ImmSrcD = immJ;    //JAL
            
            7'b0010111,    //AUIPC
            7'b0110111: ImmSrcD = immU;    //LUI
            default:    ImmSrcD = immI;          
        endcase
    end           
    
    
    
endmodule





module D_registerFile(
    input clk, rst,
    input We3,          // write enable
    input [4:0]     Rs1D, Rs2D,  
    input [4:0]     RdW,    // register destination write
    input [31:0]    Wd3,    // data ghi
    output [31:0]   Rd1, Rd2
    );
    
//==========tao register file 32x32==========
    reg [31:0] RF [31:0];

//==========khoi tao gia tri cho cac register khi rst========
    // WRITE PORT (synchronous)
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                RF[i] <= 32'b0;
        end else if (We3 && RdW != 0) begin
            RF[RdW] <= Wd3;
        end
    end
   
//=========read combinational=============
    assign Rd1 = (Rs1D == 5'b0) ? 32'b0 :
                 (We3 && Rs1D == RdW) ? Wd3 :       // khi read va write cung luc tai cung 1 RF
                  RF[Rs1D];
    assign Rd2 = (Rs2D == 5'b0) ? 32'b0 : 
                 (We3 && Rs2D == RdW) ? Wd3 :
                  RF[Rs2D];
                   
endmodule





module D_Extender(
    input [2:0] ImmSrcD,
    input [31:0] InstrD,
    output reg [31:0] ImmExtD
    );
//=========localparam cho immSrcD===========   
    localparam immI = 3'b000;
    localparam immS = 3'b001;
    localparam immB = 3'b010;
    localparam immU = 3'b011;
    localparam immJ = 3'b100;
    
    
 always @(*) begin
    case (ImmSrcD) 
        immI: ImmExtD = {{20{InstrD[31]}}, InstrD[31:20]};
        
        immS: ImmExtD = {{20{InstrD[31]}}, InstrD[31:25], InstrD[11:7]};
        
        immB: ImmExtD = {{19{InstrD[31]}}, InstrD[31], 
                             InstrD[7], InstrD[30:25], 
                             InstrD[11:8], 1'b0};
                             
        immU: ImmExtD = {InstrD[31:12], 12'b0};
        immJ: ImmExtD = {{11{InstrD[31]}}, InstrD[31],
                              InstrD[19:12], InstrD[20],
                              InstrD[30:21], 1'b0};
        default: ImmExtD = 32'b0;                      
                     
    endcase
        
 end   
    
endmodule 



