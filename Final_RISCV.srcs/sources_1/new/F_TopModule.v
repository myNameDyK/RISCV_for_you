`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/19/2026 11:01:46 PM
// Design Name: 
// Module Name: F_TopModule
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


module F_TopModule(
    input wire clk, rst, stallF, flushD,
    
    input wire          PCSrcE,        //input cho pcMuxF - lua chon pc+4 hay branch/jal
    input wire [31:0]   PCTargetE,    //input cho pcMuxF - add nhay cho lenh branch/jal
    
    output wire [31:0]  InstrD,      //output cho Fecth_stage_register - output ra instruction da fetch, gui den decode
    output wire [31:0]  PCD,         //output cho Fecth_stage_register - output ra pc, gui den decode
    output wire [31:0]  PCPlus4D    //output cho Fecth_stage_register - output ra pc+4, gui den decode

    
);
    
    wire [31:0]         InstrF;            //day dan cho imem - output ra instruction da fetch dung trong fetch stage
    wire [31:0]         PCMuxF;            //day dan cho pcMux- output ra pcMux cho pc
    wire [31:0]         PCPlus4F;          //pc + 4
    wire [31:0]         PCF;

    //THANH GHI LUU TAM PC VI IMEM DOC DONG BO NEN TRE 1 CLK
    reg [31:0] PCF_q;
    always @(posedge clk or posedge rst) begin
        if (rst)
            PCF_q <= 32'b0;
        else if (!stallF)
            PCF_q <= PCF;
    end


    // PC Adder
    F_pcAdder u_pcAdder (
        .PCF        (PCF), 
        .PCPlus4F   (PCPlus4F)
    );
    
     // PC Mux
    F_pcMux u_pcMux (
        .PCPlus4F   (PCPlus4F),
        .PCTargetE  (PCTargetE),
        .PCSrcE     (PCSrcE),
        .PCMuxF     (PCMuxF)
    );
           
    // Program Counter
    F_programCounter u_pc (
        .clk        (clk), 
        .rst        (rst), 
        .stallF     (stallF),
        .PCMuxF     (PCMuxF),
        .PCF        (PCF)
    );


    // Instruction Memory
    F_IMEM u_imem (
        .clk(clk),
        .PCF(PCF),
        .InstrF(InstrF)
    );

    
    //Fetch_Stage_Register
    Fetch_Stage_Register u_fetch (
        .clk        (clk), 
        .rst        (rst), 
        .stallF     (stallF),
        .flushD     (flushD),
    
        .PCF_q      (PCF_q),
        .InstrF     (InstrF),
    
        .PCD        (PCD),
        .PCPlus4D   (PCPlus4D),
        .InstrD     (InstrD)
    );
endmodule






module Fetch_Stage_Register(
    input clk, rst,
    input stallF,
    input flushD,

    input [31:0] PCF_q,
    input [31:0] InstrF,

    output reg [31:0] PCD,
    output reg [31:0] PCPlus4D,
    output reg [31:0] InstrD
);

    always @(posedge clk or posedge rst) begin
        if (rst || flushD) begin
            PCD      <= 32'b0;
            PCPlus4D <= 32'b0;
            InstrD   <= 32'b0;   // NOP
        end
        else if (!stallF) begin
            PCD      <= PCF_q;
            PCPlus4D <= PCF_q + 32'd4;
            InstrD   <= InstrF;
        end
    end
endmodule







module F_IMEM(
    input   clk,
    input   wire [31:0]  PCF,  
    output reg [31:0]  InstrF
    );
    
    (* ram_style = "block" *)
    reg [31:0] IMEM [0:1023]; // 1024 word × 32 bit = 32 Kb

//    initial begin
//        $readmemh("D:/Everything_with_VIVADO/VIVADO_PROJECT/Final_RISCV/filemem.hex", MEM);
//    end 
    integer i;
    initial begin
        // Fill NOP ?? tránh X
        for (i = 0; i < 256; i = i + 1)
            IMEM[i] = 32'h00000013; // addi x0,x0,0

   
        $readmemh("filemem.hex", IMEM);
    end
    
    always @(posedge clk) begin
        InstrF <= IMEM[PCF[11:2]];  // PC >> 2,  10 bit
    end
endmodule







module F_pcAdder(
        input wire  [31:0]  PCF,
        
        output wire [31:0]  PCPlus4F
        );

    assign PCPlus4F = PCF + 32'd4;
endmodule






module F_pcMux(
        input wire [31:0]   PCPlus4F,
        input wire [31:0]   PCTargetE,  // add nhay cho cac lenh branch/jal/jalr
        input wire          PCSrcE,      // lua chon giua pc+4 or pctarget
        
        output     [31:0]   PCMuxF      //output pcnext cho PC block
        );

assign PCMuxF = (PCSrcE == 1'b0) ?  PCPlus4F:
                (PCSrcE == 1'b1) ?  PCTargetE:
                                    PCPlus4F;
endmodule





module F_programCounter(
    input wire          clk, rst, stallF,
    
    input wire [31:0]   PCMuxF,
    
    output reg [31:0]   PCF
    );
    
    always @(posedge clk or posedge rst) begin
        if(rst)
            PCF <= 32'h0000_0000;
        else if(!stallF)
            PCF <= PCMuxF;
    end
endmodule

