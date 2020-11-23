module FLipFlopD1bit(input wire D,input wire clock,input wire reset, input wire enable,output Q);
reg Q;
	always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					Q <= 0; 
				end
			else if(enable==1) 
				begin
					Q <= D;
				end
			else
				begin
					Q <= Q;
		end
	end
endmodule 

module FLipFlopD4bits(input wire [3:0] D,input wire clock,input wire reset, input wire enable,output [3:0] Q);
reg Q;
	always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					Q <= 4'b0000; 
				end
			else if(enable==1) 
				begin
					Q <= D;
				end
			else
				begin
					Q <= Q;
		end
	end
endmodule 

module FLipFlopT (input wire clock, input wire reset, output Q);
reg Q;
	always @(posedge reset or posedge clock)
		begin
			if(reset==1)
				Q <= 0;
			else
				Q <= ~Q;
		end
	
endmodule

module BufferTriEstado4bits (input wire enable, input wire [3:0 ] IN, output [3:0] OUT); 

	assign OUT = enable ? IN : 4'bz;

endmodule

//Módulo de Program Counter
module ProgramCounter(input wire clock, reset, incPC, loadPC,input wire [11:0]address_RAM,output reg [11:0]PC);

        always @ (posedge clock or posedge reset)begin 
            if (reset == 1)
                PC <= 12'b000000000000;
            
            else if(loadPC == 1)
                PC <= address_RAM;

            else if(incPC == 1 && ~loadPC) 
            PC <= PC + 1;
        end
endmodule

//Módulo de la Memoria ROM
module MemoriaROM (input wire [11:0]PC, output wire [7:0]program_byte);           
	reg [11:0] ROM [0:4095] ;  
    
	assign program_byte = ROM[PC];

	initial begin
	  $readmemh("memory.list", ROM); 
	end

endmodule

//Módulo de Fetch
module Fetch (input wire [7:0] program_byte, input wire clock, input wire reset, input wire no_phase, output [3:0] instr, output [3:0] oprnd);

	FLipFlopD4bits FFD_I(program_byte[7:4], clock, reset, no_phase, instr);
	FLipFlopD4bits FFD_O(program_byte[3:0], clock, reset, no_phase, oprnd);

endmodule

//Módulo Phase
module Phase (input wire clock, input wire reset, output phase);

	FLipFlopT Phase (clock, reset, phase);
	
endmodule

//Módulo Flags
module Flags (input wire C, input wire Z, input wire clock, input wire reset, input wire loadFlags, output c_flag, output z_flag);

	FLipFlopD1bit FFD_C (C, clock, reset, loadFlags, c_flag);
	FLipFlopD1bit FFD_Z (Z, clock, reset, loadFlags, z_flag);
	
endmodule

//Módulo Decode
module Decode( input wire [6:0] decode_in, output [12:0] decode_out);
    
    reg [12:0] decode_out;
    
    always @ (decode_in)
        casex(decode_in)
            // any
            7'bxxxx_xx0: decode_out <= 13'b1000_000_001000;
            // JC
            7'b0000_1x1: decode_out <= 13'b0100_000_001000;
            7'b0000_0x1: decode_out <= 13'b1000_000_001000;
            // JNC
            7'b0001_1x1: decode_out <= 13'b1000_000_001000;
            7'b0001_0x1: decode_out <= 13'b0100_000_001000;
            // CMPI
            7'b0010_xx1: decode_out <= 13'b0001_001_000010;
            // CMPM
            7'b0011_xx1: decode_out <= 13'b1001_001_100000;
            // LIT
            7'b0100_xx1: decode_out <= 13'b0011_010_000010;
            // IN
            7'b0101_xx1: decode_out <= 13'b0011_010_000100;
            // LD
            7'b0110_xx1: decode_out <= 13'b1011_010_100000;
            // ST
            7'b0111_xx1: decode_out <= 13'b1000_000_111000;
            // JZ
            7'b1000_x11: decode_out <= 13'b0100_000_001000;
            7'b1000_x01: decode_out <= 13'b1000_000_001000;
            // JNZ
            7'b1001_x11: decode_out <= 13'b1000_000_001000;
            7'b1001_x01: decode_out <= 13'b0100_000_001000;
            // ADDI
            7'b1010_xx1: decode_out <= 13'b0011_011_000010;
            // ADDM
            7'b1011_xx1: decode_out <= 13'b1011_011_100000;
            // JMP
            7'b1100_xx1: decode_out <= 13'b0100_000_001000;
            // OUT
            7'b1101_xx1: decode_out <= 13'b0000_000_001001;
            // NANDI
            7'b1110_xx1: decode_out <= 13'b0011_100_000010;
            // NANDM
            7'b1111_xx1: decode_out <= 13'b1011_100_100000;
            default: decode_out <= 13'b1111111111111;
        endcase
endmodule

//Módulo Bus de datos 1
module Buffer1 (input wire oeOprnd, input wire [3:0] oprnd, output [3:0] data_bus); 

BufferTriEstado4bits B1 (oeOprnd, oprnd, data_bus); 

endmodule

//Módulo Bus de datos 2
module Buffer2 (input wire oeALU, input wire [3:0] alu_out, output [3:0] data_bus); 

BufferTriEstado4bits B2 (oeALU, alu_out, data_bus); 

endmodule

//Módulo Bus de datos 3
module Buffer3 (input wire oeIN, input wire [3:0] pushbuttons, output [3:0] data_bus); 

BufferTriEstado4bits B3 (oeIN, pushbuttons, data_bus); 

endmodule

//Módulo Memoria RAM
module MemoriaRAM(input wire csRAM, weRAM, input wire [11:0]address_RAM, inout wire [3:0]data_bus);

    reg [3:0]RAM[0:4095]; 
    reg [3:0]databus;
    assign data_bus = (csRAM && ~weRAM) ? databus: 4'bzzzz; 

    always @(csRAM, weRAM, address_RAM, data_bus) begin 
        
        if (csRAM && ~weRAM)
            databus= RAM[address_RAM];
        if (csRAM && weRAM)
            RAM[address_RAM] = data_bus; 
            
        end
endmodule

//Módulo Acumulador
module Accu (input wire [3:0] alu_out,input wire clock,input wire reset, input wire loadA,output [3:0] accu);

	FLipFlopD4bits FFD_Accu (alu_out, clock, reset, loadA, accu);
	
endmodule

//Módulo ALU
module ALU (input wire [3:0] accu, data_bus, input [2:0] sel, output C, Z, output [3:0] alu_out);
    
    reg [4:0] q;
    
    always @ (accu, data_bus, sel)
        case (sel)
            3'b000: q = accu; 
            3'b001: q = accu - data_bus; 
            3'b010: q = data_bus; 
            3'b011: q = accu + data_bus; 
            3'b100: q = {1'b0, ~(accu & data_bus)};
            default: q = 5'b10101;
        endcase
    
    assign alu_out = q[3:0];
    assign C = q[4];
    assign Z = ~(q[3] | q[2] | q[1] | q[0]);
    
endmodule

//Módulo Outputs
module Outputs (input wire [3:0] data_bus,input wire clock,input wire reset, input wire loadOut,output [3:0] FF_out);

	FLipFlopD4bits FFD_Out (data_bus, clock, reset, loadOut, FF_out);
	
endmodule

//Módulo Principal
module uP( input wire clock, reset, input wire [3:0]pushbuttons, output wire phase, c_flag, z_flag, output wire [3:0] instr, oprnd, accu, data_bus, FF_out, output wire [7:0] program_byte, output wire [11:0] PC, address_RAM);

wire [3:0]  alu_out;
wire [12:0] decode_out;
wire [6:0]  decode_in;
wire [2:0]  sel;
wire Z, C;

assign address_RAM = {oprnd, program_byte};
assign decode_in = {instr, c_flag, z_flag, phase};
assign incPC = decode_out [12];
assign loadPC = decode_out [11];
assign loadA = decode_out [10];
assign loadFlags = decode_out [9];
assign sel = decode_out [8:6];
assign csRAM = decode_out [5];
assign weRAM = decode_out [4];
assign oeALU = decode_out [3];
assign oeIN = decode_out [2];
assign oeOprnd = decode_out [1];
assign loadOut = decode_out [0];



ProgramCounter	PORGCOUNT	(clock, reset, incPC, loadPC, address_RAM, PC);
MemoriaROM		ROM			(PC, program_byte);
Fetch 			FETCH		(program_byte, clock, reset, ~phase, instr, oprnd);
Phase 			PHASE		(clock, reset, phase);
Flags 			FLAGS		(C, Z, clock, reset, loadFlags, c_flag, z_flag);
Decode			DECODE		(decode_in, decode_out);
Buffer1 		BTE1		(oeOprnd, oprnd, data_bus); 
Buffer2 		BTE2		(oeALU, alu_out, data_bus); 
Buffer3 		BTE3		(oeIN, pushbuttons, data_bus); 
MemoriaRAM		RAM			(csRAM, weRAM, address_RAM, data_bus);
Accu 			ACCU		(alu_out, clock, reset, loadA, accu);
ALU 			ALU			(accu, data_bus, sel, C, Z, alu_out);
Outputs 		OUTPUT		(data_bus,clock,reset, loadOut,FF_out);

endmodule