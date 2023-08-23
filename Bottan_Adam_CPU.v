module top (	input         clk, reset,
		output [31:0] data_to_mem, address_to_mem,
		output        write_enable);

	wire [31:0] pc, instruction, data_from_mem;

	inst_mem  imem(pc[7:2], instruction);
	data_mem  dmem(clk, write_enable, address_to_mem, data_to_mem, data_from_mem);
	processor CPU(clk, reset, pc, instruction, write_enable, address_to_mem, data_to_mem, data_from_mem);
endmodule

//-------------------------------------------------------------------
module data_mem (input clk, we,
		 input  [31:0] address, wd,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];

	initial begin
		$readmemh ("test.txt",RAM,0,63);
	end

	assign rd=RAM[address[31:2]]; // word aligned

	always @ (posedge clk)
		if (we)
			RAM[address[31:2]]<=wd;
endmodule

//-------------------------------------------------------------------
module inst_mem (input  [5:0]  address,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];
	initial begin
		$readmemh ("Bottan_Adam_prog1.hex",RAM,0,63);
	end
	assign rd=RAM[address]; // word aligned
endmodule
module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
                
                wire BranchBeq, BranchJal, BranchJalr, RegWrite, MemToReg, ALUSrc, zero, Auipc;
                wire [31:0] pcnext, SrcA, SrcB, result, mux, ImmOp, PCPlus4, pcbr, BranchTarget, WD3;
                wire [2:0] immControl;
                wire [3:0] ALUControl;
                
                // Control Unit
                ControlUnit PCU( instruction[31:0], BranchBeq, BranchJal, BranchJalr, RegWrite, 
                		   MemToReg, WE, ALUSrc, Auipc, immControl, ALUControl);
                		   
                // Register
                Reg_32 preg( instruction[19:15], instruction[24:20], instruction[11:7], WD3, clk, RegWrite, SrcA, data_to_mem);
                ImmDecode immDec( instruction[31:7], immControl, ImmOp);
                MUX_2_1_32 rd2immOp( ImmOp, data_to_mem, ALUSrc, SrcB);
                MUX_2_1_32 resAdd(pcbr, result, Auipc, WD3);
                
                //PC logic
                Reg_res res_reg(pcnext, clk, reset, PC);
                Adder_32 pc_adder(PC, 32'd4, PCPlus4);
                Adder_32 immPC(ImmOp, PC, pcbr);
                MUX_2_1_32 pc_mux(BranchTarget, PCPlus4, ((zero & BranchBeq) | (BranchJal | BranchJalr)), pcnext);
                
                //ALU logic
                ALU alu( SrcA, SrcB, ALUControl, address_to_mem, zero);
                MUX_2_1_32 result_mux(data_from_mem, mux, MemToReg, result);
                MUX_2_1_32 aluAdd(address_to_mem, pcbr, BranchJalr, BranchTarget);
                MUX_2_1_32 pcAddAlu(PCPlus4, address_to_mem, (BranchJal | BranchJalr), mux);
                
endmodule

//------------------------------modules----------------------------------//

module Adder_32( input signed [31:0] a,b, output [31:0] sum );
	assign sum = a+b;
endmodule

module MUX_2_1_32(input [31:0] d0, d1, input select, output [31:0] y );
	
	assign y = select?d0:d1;

endmodule

module Reg_32(input [4:0] a,b,c, input [31:0] wd3, input clk, we3, output [31:0] rd1, rd2);

	reg [31:0] rf[31:0];
	
	always@(posedge clk) 
		if ( we3 ) rf[c] = wd3;
		
	assign rd1 = (a != 0) ? rf[a] : 0;
	assign rd2 = (b != 0) ? rf[b] : 0;

endmodule

module Reg_res(input [31:0] a0, input clk, reset, output reg [31:0] rd1);

	always@(posedge clk, posedge reset) begin
		if( reset )
			rd1 = 0;
		else
			rd1 = a0;
	end

endmodule

module ImmDecode(input [31:7] instr, input [2:0] immCont, output reg [31:0] immExt);

	always @(*)
		case(immCont)
			3'b000:  immExt = {{21{instr[31]}}, instr[30:20]}; 					// I-type
			3'b001:  immExt = {{21{instr[31]}}, instr[30:25], instr[11:7]};			// S-type
			3'b010:  immExt = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};	// B-type
			3'b011:  immExt = {instr[31:12], 12'b0};						// U-type
			3'b100:  immExt = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};	// J-type
			default: immExt = 32'bx;
		endcase

endmodule

module ALU(input signed [31:0] srca, srcb, input [3:0] alucontr, output reg [31:0] alures, output reg zero);

	always@(*) begin
		case( alucontr )
			4'b0000: alures = srca + srcb;
			4'b0001: alures = srca - srcb;
			4'b0010: alures = srca / srcb;
			4'b0011: alures = srca % srcb;
			4'b0100: alures = srca & srcb;
			4'b0101: alures = srcb;
			4'b0110: alures = srcb <= srca;
			4'b0111: alures = srca < srcb;
			4'b1000: alures = srca << srcb;
			4'b1001: alures = srca >>> srcb;
			4'b1010: alures = srca >> srcb;
		endcase
		
		zero <= ((alures == 0) ? ( 1 ) : 0); 
	end

endmodule

module ControlUnit(input [31:0] instr, output brbq, brjal, brjalr, regwr, memtoreg, memwr, alusrc, auipc, output [2:0] immcontr, output [3:0] alucontr);

	reg [14:0] controls;
	
	assign {regwr, immcontr, alusrc, brbq, brjal, brjalr, memwr, memtoreg, alucontr, auipc} = controls;
	

	always@(*) 
	case ( instr[6:0] )
		7'b0110011: case ( instr[31:25] ) // ADD, AND, SUB, SLT, DIV, REM  
				7'b0000000: case ( instr[14:12] )
						3'b000: controls = 15'b1_xxx_0_0_0_0_0_0_0000_0 ; // ADD
						3'b111: controls = 15'b1_xxx_0_0_0_0_0_0_0100_0 ; // AND
						3'b010: controls = 15'b1_xxx_0_0_0_0_0_0_0111_0 ; // SLT
						3'b001: controls = 15'b1_xxx_0_0_0_0_0_0_1000_0 ; // SLL
						3'b101: controls = 15'b1_xxx_0_0_0_0_0_0_1010_0 ; // SRL
					    endcase
				7'b0100000: case ( instr[14:12] ) 
						3'b000: controls = 15'b1_xxx_0_0_0_0_0_0_0001_0 ; // SUB
						3'b101: controls = 15'b1_xxx_0_0_0_0_0_0_1001_0 ; // SRA
					    endcase	
				7'b0000001: case ( instr[14:12] )
						3'b100: controls = 15'b1_xxx_0_0_0_0_0_0_0010_0 ; // DIV
						3'b110: controls = 15'b1_xxx_0_0_0_0_0_0_0011_0 ; // REM
					    endcase
			    endcase
		7'b0010011: controls = 15'b1_000_1_0_0_0_0_0_0000_0 ; // ADDI
		7'b0000011: controls = 15'b1_000_1_0_0_0_0_1_0000_0 ; // LW
		7'b0100011: controls = 15'b0_001_1_0_0_0_1_1_0000_0 ; // SW
		7'b1100011: case ( instr[14:12] ) // BEQ, BLT
				3'b000: controls = 15'b0_010_0_1_0_0_0_x_0001_0 ; // BEQ
				3'b100: controls = 15'b0_010_0_1_0_0_0_x_0110_0 ; // BLT
			    endcase
		7'b1101111: controls = 15'b1_100_x_0_1_0_0_0_0000_0 ; // JAL
		7'b1100111: controls = 15'b1_000_1_0_0_1_0_0_0000_0 ; // JALR
		7'b0110111: controls = 15'b1_011_1_0_0_0_0_0_0101_0 ; // LUI
		7'b0010111: controls = 15'b1_011_x_0_0_0_0_0_0101_1 ; // AUIPC
		default: controls = 15'bx ;
	endcase
		

endmodule
