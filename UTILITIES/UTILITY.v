`timescale 1ns / 1ps

module UTILITY(
	input 		  clk,
	input 		  rst,
	input 		  enable_int,
	input [31:0]  imm,
	input [31:0]  irr_ret,
	input [31:0]  irr_dest,
	input 		  irr,
	input [11:0]  opcode,
	input [31:0]  rs1,
	input 		  branch, 
	output [31:0] rd,
	output [31:0] pc);

	reg [63:0] N_CYCLE=0,N_INSTRUC=0,REAL_TIME=0;
	reg [31:0] TIME=0,rd_n=0,PC_N=0,PC_N2=0,RD_DATA=0;
	wire [31:0] PC_BRANCH, PC_SALTOS, PC_ORIG;
	reg is_rd;

	// RDCYCLE REGISTER
	always @(posedge clk )
		begin
			if (rst) N_CYCLE<=0;
			else  N_CYCLE <= N_CYCLE + 1;
		end

	// REAL TIME REGISTER
	always @(posedge clk )
		begin
			if (rst) begin
				TIME<=0;
				REAL_TIME<=0;
			end else if(TIME==100) begin
				TIME<=0;
				REAL_TIME<=REAL_TIME+1;
			end else 
				TIME <= TIME + 1;
		end	
	
	// INSTRUCTION NUMBER REGISTER
	always @(posedge clk )
		begin
			if (rst) N_INSTRUC<=0;
			else if (enable_int)  N_INSTRUC <= N_INSTRUC + 1;	
		end


	// RD assignation phase 1: CSRR* Assigns
	always @(imm, N_CYCLE, REAL_TIME, N_INSTRUC)
		case (imm)
			32'b00000000000000000000110010000000: RD_DATA = N_CYCLE[63:32];		// RDCYCLEH
			32'b00000000000000000000110000000000: RD_DATA = N_CYCLE[31:0];		// RDCYCLE
			32'b00000000000000000000110010000001: RD_DATA = REAL_TIME[63:32];	// RDTIMEH
			32'b00000000000000000000110000000001: RD_DATA = REAL_TIME[31:0];	// RDTIME
			32'b00000000000000000000110010000010: RD_DATA = N_INSTRUC[63:32];	// RDINSTRETH
			32'b00000000000000000000110000000010: RD_DATA = N_INSTRUC[31:0];	// RDINSTRET
			default: RD_DATA = 0;												// Other CSRR* (Read zero)
		endcase
	
	// RD assignation phase 2: instruction-specific
	always @(opcode, RD_DATA,PC_N,imm)
		is_rd = 1;
		case (opcode)
			7'b000001110011 : rd_n = RD_DATA;		// CSRR*
			7'b000001101111 : rd_n = PC_ORIG;		// JAL
			7'b000001100111 : rd_n = PC_ORIG;		// JALR
			7'b000000010111 : rd_n = PC_N2+imm;		// AUIPC
			default: begin rd_n = 0; is_rd = 0; end	// ILLEGAL
		endcase
	
	assign rd = is_rd?rd_n:32'hzzzzzzzz;

	// Next PC Determination
	assign PC_SALTOS = PC_N2 + imm;
	assign PC_ORIG = PC_N2 + 4;
	assign PC_BRANCH = branch ? PC_SALTOS : PC_ORIG;
	always @*
		if(irr) begin
			PC_N = irr_pc;								// An interrupt (Who saves the previous PC is IRQ module via irr_ret)
		end else if(opcode[6:0] == 7'b1100011) begin
			PC_N = PC_BRANCH;							// BXX 
		end else begin
			case (opcode)
				7'b000001100111: PC_N = rs1;			// JALR
				7'b000001101111: PC_N = PC_SALTOS;		// JAL
				7'b001110011000: PC_N = irr_ret;		// RETIRQ
				default: 		 PC_N = PC_ORIG;		// Advance the program counter
			endcase
		end		

	// PC sync
	always @(posedge clk)
		begin	
			if (rst) PC_N2<=0;
			else if(enable_int) PC_N2 <= PC_N ;
		end
	assign pc = PC_N2;
endmodule