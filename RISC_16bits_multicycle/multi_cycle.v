// Mehul Kavdia - 2018A8PS0860P
// Anubhav Srivastava - 2018A8PS0030P

////////////////////////////////Test bench//////////////////////////////
module tb_multi_cycle();
	wire address_invalid,result_is_zero;
	reg clk,reset;

	multi_cycle uut(result_is_zero,address_invalid,clk,reset);
	initial 
		#1805 $finish;//first posedge at t=5, Total time = #cycles*10 + 5
	initial begin 
		clk=1'b0;
		forever #5 clk=~clk;
	end
	initial begin
	reset=1'b0;
	#2 reset=1'b1;
	#1 reset=1'b0;
	//IF starts from t = 5
	end
	initial begin 
		//$monitor("time = %t, address_invalid = %b, clk=%b, reset=%b", $time,address_invalid,clk,reset);
		$monitor("ALU calculation result_is_zero = %b address_invalid = %b\n",result_is_zero,address_invalid);
	end
endmodule

/////////////////////////////////INSTRUCTION MEMORY///////////////////////////////////////////
module inst_mem(inst,address_invalid,address,instread);
	output reg [15:0]inst;
	output reg address_invalid;
	input [15:0]address;
	input instread;

	reg [7:0]storage_inst[0:65535]; //2^16 locations, byte organised

	initial $readmemh("instr.dat",storage_inst);
	
	always @(instread,address)
		begin
		address_invalid = 0;
		if(instread) 
			begin
			inst[7:0]<=storage_inst[address];  //Little endian
			inst[15:8]<=storage_inst[address+1];
			end
		if(address[0]==1)	
			address_invalid = 1;
		end
		
endmodule
///////////////////////////////DATA MEMORY///////////////////////////////
module data_mem(data,address,writedata,memread,memwrite,clk);
	output reg [15:0]data;
	input [15:0]writedata,address;
	input memread,memwrite,clk;

	reg [7:0]storage_mem[0:65535]; //2^16 locations, byte organised

	initial $readmemh("data.dat",storage_mem);

	always @(memread,address)
		begin
		if(memread) 
			begin
			data[7:0]<=storage_mem[address]; //Little endian
			data[15:8]<=storage_mem[address+1];
			end
		end

	always @(negedge clk)
		begin
		if(memwrite)
			begin
			storage_mem[address]=writedata[7:0]; //Little Endian
			storage_mem[address+1]=writedata[15:8];
			$display("Writing in Data memory at address: %h\n *********Store instruction Completes*********\n",address);
			$writememh("data.dat",storage_mem);
			end
		end

endmodule

/////////////////////////////////INSTRUCTION REGISTER///////////////////////////////
module inst_reg(rb_rs2,ra_rs1,rt_rd,rp,rd,imm8bit,imm12bit,opcode,inst,irwrite,clk);
	output reg [3:0]rb_rs2,ra_rs1,rt_rd;
	output reg [1:0]rp,rd;
	output reg [7:0]imm8bit;
	output reg [11:0]imm12bit;
	output reg [3:0]opcode;
	input [15:0]inst;
	input irwrite,clk;

	always @(negedge clk)
		begin
		if(irwrite)
			begin
			rb_rs2<=inst[3:0]; 
			ra_rs1<=inst[7:4];
			rt_rd<=inst[11:8];
			rp<=inst[9:8];
			rd<=inst[11:10];
			imm8bit<=inst[7:0];
			imm12bit<=inst[11:0];
			opcode<=inst[15:12];
			end
		end

endmodule

//////////////////////////////////////REGISTER FILE///////////////////////////////

module reg_file(readdata1,readdata2,readdata3,readdata4,readdata5,readreg1,readreg2,readreg3,readreg4,readreg5,writereg,writedata,regwrite,clk);
	output reg [15:0]readdata1,readdata2,readdata3,readdata4,readdata5;
	input [3:0]readreg1,readreg2,readreg3,readreg4,readreg5,writereg;
	input [15:0]writedata;
	input regwrite,clk;
	reg [15:0]registers[0:15]; //16 registers and each register is 16 bit long

	initial 
		begin
			$display("R0=%h, R1=%h, R2=%h, R3=%h, R4=%h, R5=%h, R6=%h, R7=%h, R8=%h, R9=%h, R10=%h, R11=%h, R12=%h, R13=%h, R14=%h, R15=%h\n",registers[0],registers[1],registers[2],registers[3],registers[4],registers[5],registers[6],registers[7],registers[8],registers[9],registers[10],registers[11],registers[12],registers[13],registers[14],registers[15]);
		end
		
	always @(*)
		begin
			readdata1<=registers[readreg1];
			readdata2<=registers[readreg2];
			readdata3<=registers[readreg3];
			readdata4<=registers[readreg4];
			readdata5<=registers[readreg5];
			registers[0]<=0;
		end

	always @(negedge clk)
		begin
			if(regwrite)
			begin
				registers[writereg]=writedata; 
				$display("Writing in Register:R%d \n*********Instruction Completes*********\n",writereg);
				$display("R0=%h, R1=%h, R2=%h, R3=%h, R4=%h, R5=%h, R6=%h, R7=%h, R8=%h, R9=%h, R10=%h, R11=%h, R12=%h, R13=%h, R14=%h, R15=%h\n",registers[0],registers[1],registers[2],registers[3],registers[4],registers[5],registers[6],registers[7],registers[8],registers[9],registers[10],registers[11],registers[12],registers[13],registers[14],registers[15]);
			end	
		end
endmodule

////////////////////////////////Program Counter///////////////////////////////

module pc(pcout,pcin,y_or_pcwrite,clk,reset);
	output reg [15:0]pcout;
	input [15:0]pcin;
	input y_or_pcwrite,clk,reset;
	
	always @(negedge clk,posedge reset)
		begin
			if(reset) 
				pcout<=16'd0;
			else
			begin
				if(y_or_pcwrite) 
					begin
					pcout<=pcin; 
					end
			end

		end
endmodule

////////////////////////////////register 16 bits--for MDR,A,B,C,D,E,aluout///////////////////////////////

module reg_16bits(out,in,clk,reset);
	output reg [15:0]out;
	input [15:0]in;
	input clk,reset;

	always @(posedge clk)
		begin
		if(reset) 
			out<=16'd0;
		else 
			out<=in;
		end
endmodule

module reg_16bits1(out,in,clk,reset);
	output reg [15:0]out;
	input [15:0]in;
	input clk,reset;

	always @(posedge clk)
		begin
		if(reset) 
			out<=16'd0;
		else 
		begin
			out=in;
			$display("ALUout= %h\n",out);
			end
		end
endmodule

/////////////////////////////ALU///////////////////////////////

module alu(result,zero,aluin1,aluin2,operation);
	output reg [15:0]result;
	output reg zero;
	input signed [15:0]aluin1,aluin2;
	input [7:0]operation;

	always @(*)
		begin
		casex(operation)
		8'bxxxx_1000: result<=aluin1+aluin2; //add registers sources,IF
		8'bxxxx_1001: result<=aluin1+{{8{aluin2[7]}},aluin2[7:0]}; //addimm with imm value sign extended to 16 bits
		8'bxxxx_1010: result<=aluin1+aluin2; //addimm with imm value already extended to 16 bits by 0s

		8'bxxxx_1100: result<=aluin2-aluin1; //subtract registers sources 
		8'bxxxx_1101: result<=aluin1-{{8{aluin2[7]}},aluin2[7:0]}; //subimm with imm value sign extended to 16 bits
		8'bxxxx_1110: result<=aluin1-aluin2; //subimm with imm value already extended to 16 bits by 0s

		8'b0001_0000: result<=aluin1<<aluin2[7:4]; //SLL
		8'b0010_0000: result<=aluin1>>aluin2[7:4]; //SRL
		8'b0011_0000: result<=aluin1>>>aluin2[7:4]; //SAR

		8'bxxxx_1011: result<= ~(aluin1 & aluin2); //NAND
		8'bxxxx_1111: result<=aluin1 | aluin2; //OR
		8'bxxxx_0111: result<=~(aluin1 & ({{8{aluin2[7]}},aluin2[7:0]})); //NANDimm with imm value sign extended to 16 bits
		8'bxxxx_0110: result<=aluin1 | ({{8{aluin2[7]}},aluin2[7:0]}); //ORimm with imm value sign extended to 16 bits

		8'bxxxx_0100: result<=aluin1-aluin2;//BEQ
		8'bxxxx_0101: result<=aluin1-aluin2;//BNEQ

		8'bxxxx_0011: result<=aluin1+aluin2;//Jump-address calculation

		8'bxxxx_0001: result<=aluin1+{{7{aluin2[7]}},aluin2[7:0],1'b0}; //LW-- memory address calculation--Rp+imm(sign extended to 16 bits and left shifted by 1 bit
		8'bxxxx_0010: result<=aluin1+{{7{aluin2[7]}},aluin2[7:0],1'b0}; //SW-- memory address calculation--Rp+imm(sign extended to 16 bits and left shifted by 1 bit

		default: result<=16'bxxxx_xxxx_xxxx_xxxx;

		endcase
		end

	always @(*)
		begin
		if(result==16'd0) 
			zero<=1'b1; //if result is 0 then zero=1 else zero=0
		else 
			zero<=1'b0;
		end
endmodule

///////////////////////////////////CONTROL UNIT///////////////////////////////

module control_unit(memwrite,Regdst,MemtoReg,RegWrite,MemRead,IrWrite,
					InstrRead,AluSrcA,AluSrcB,AluControlMux,PcWrite,
					PcWriteCond1,PcWriteCond2,PcSrc,clk,opcode,rst);
	
	input clk,rst;
	input [3:0] opcode;
	
	output reg memwrite,Regdst,MemtoReg,RegWrite,MemRead,IrWrite,InstrRead,AluControlMux,PcWrite,PcWriteCond1,PcWriteCond2;
	output reg[1:0] AluSrcA,AluSrcB,PcSrc;
	
	localparam S0 = 4'b0000, S1 = 4'b0001, S2 = 4'b0010, S3 = 4'b0011, S4 = 4'b0100,
	S5 = 4'b0101, S6 = 4'b0110, S7 = 4'b0111, S8 = 4'b1000, S9 = 4'b1001, S10 = 4'b1010, S11 = 4'b1011,idle = 4'b1100;
	
	reg [3:0] present_state, next_state;		
	
	always@(posedge clk,posedge rst)
		begin
			if(rst)
				present_state<=idle;
			else
				present_state<=next_state;
		end
	
	always@(*)
		begin
			memwrite = 0;
			Regdst = 0;
			MemtoReg = 0;
			RegWrite = 0;
			MemRead = 0;
			IrWrite = 0;
			InstrRead = 0;
			AluControlMux = 0;
			PcWrite = 0;
			PcWriteCond1 = 0;
			PcWriteCond2 = 0;
			AluSrcA = 0;
			AluSrcB = 0;
			PcSrc = 0;
			case(present_state)
				idle: begin
						memwrite = 1'bx;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 1'bx;
						MemRead = 1'bx;
						IrWrite = 1'bx;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 1'bx;
						PcWriteCond1 = 1'bx;
						PcWriteCond2 = 1'bx;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
					end
				S0:	begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 1;
						InstrRead = 1;
						AluControlMux = 0;
						PcWrite = 1;
						PcWriteCond1 = 1'bx;
						PcWriteCond2 = 1'bx;
						AluSrcA = 2'b10;
						AluSrcB = 2'b11;
						PcSrc = 2'b00;
					end
				S1: begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
						$display("ID: Instruction decoded with opcode %b\n",opcode);
					end
				S2:	begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'b11;
						AluSrcB = 2'b01;
						PcSrc = 2'bxx;
						case(opcode)
							4'b0001: $display("EX: Load instruction address calculation\n");
							4'b0010: $display("EX: Store instruction address calculation\n");
						endcase
					end
				S3: begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'b01;
						AluSrcB = 2'b01;
						PcSrc = 2'bxx;
						
						case(opcode)
							4'b1001: $display("EX: ADD immediate(sign extended) result calculation\n");
							4'b1010: $display("EX: ADD immediate(zero extended) result calculation\n");
							4'b1101: $display("EX: SUB immediate(sign extended) result calculation\n");
							4'b1110: $display("EX: SUB immediate(zero extended) result calculation\n");
							4'b0111: $display("EX: NAND immediate(sign extended) result calculation\n");
							4'b0110: $display("EX: OR immediate(sign extended) result calculation\n");
							4'b0000: $display("EX: SHIFT instruction result calculation\n");
						endcase
					end
				S4:	begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'b00;
						AluSrcB = 2'b00;
						PcSrc = 2'bxx;
						case(opcode)
							4'b1000: $display("EX: ADD(2 source) result calculation\n");
							4'b1100: $display("EX: SUB(2 source) result calculation\n");
							4'b1011: $display("EX: NAND(2 source) result calculation\n");
							4'b1111: $display("EX: OR(2 source) result calculation\n");
						endcase
					end
				S5: begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 1;
						AluSrcA = 2'b00;
						AluSrcB = 2'b00;
						PcSrc = 2'b01;
						$display("EX: Branch if not equal - if Ra!=Rb, then branch to the given address\n");
						$display("*********Branch if not equal completes*********\n");
					end
				S6:	begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 0;
						PcWriteCond1 = 1;
						PcWriteCond2 = 0;
						AluSrcA = 2'b00;
						AluSrcB = 2'b00;
						PcSrc = 2'b01;
						$display("EX: Branch if equal - if Ra==Rb, then branch to the given address\n");
						$display("*********Branch if equal completes*********\n");
					end
				S7: begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1;
						PcWrite = 1;
						PcWriteCond1 = 1'bx;
						PcWriteCond2 = 1'bx;
						AluSrcA = 2'b10;
						AluSrcB = 2'b10;
						PcSrc = 2'b10;
						$display("EX: Jump to PC+2+immediate address\n");
						$display("*********Jump completes*********\n");
					end
				S8:	begin
						memwrite = 0;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
						$display("MEM: Loads data into Memory data register from data memory\n");
					end
				S9: begin
						memwrite = 1;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 0;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
						$display("MEM: Store instruction ");
					end
				S10: begin
						memwrite = 0;
						Regdst = 1;
						MemtoReg = 1;
						RegWrite = 1;
						MemRead = 1'bx;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
						
						case(opcode)
							4'b1001: $display("MEM: ADD immediate(sign extended) ");
							4'b1010: $display("MEM: ADD immediate(zero extended) ");
							4'b1101: $display("MEM: SUB immediate(sign extended) ");
							4'b1110: $display("MEM: SUB immediate(zero extended) ");
							4'b0111: $display("MEM: NAND immediate(sign extended) ");
							4'b0110: $display("MEM: OR immediate(sign extended) ");
							4'b0000: $display("MEM: SHIFT instruction ");
							4'b1000: $display("MEM: ADD(2 source) ");
							4'b1100: $display("MEM: SUB(2 source) ");
							4'b1011: $display("MEM: NAND(2 source) ");
							4'b1111: $display("MEM: OR(2 source) ");
						endcase
					end
				S11: begin
						memwrite = 0;
						Regdst = 0;
						MemtoReg = 0;
						RegWrite = 1;
						MemRead = 1;
						IrWrite = 0;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 0;
						PcWriteCond1 = 0;
						PcWriteCond2 = 0;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
						$display("WB: Load instruction ");
					end
				default: 
					begin
						memwrite = 1'bx;
						Regdst = 1'bx;
						MemtoReg = 1'bx;
						RegWrite = 1'bx;
						MemRead = 1'bx;
						IrWrite = 1'bx;
						InstrRead = 1'bx;
						AluControlMux = 1'bx;
						PcWrite = 1'bx;
						PcWriteCond1 = 1'bx;
						PcWriteCond2 = 1'bx;
						AluSrcA = 2'bxx;
						AluSrcB = 2'bxx;
						PcSrc = 2'bxx;
					end
			endcase
		end
	
	always@(*)
		begin
			next_state = present_state;
			case(present_state)
				idle: next_state=S0;
				S0: next_state = S1;
				S1: begin
						case(opcode)
							4'b0001: next_state = S2;//Load, store
							4'b0010: next_state = S2;
							
							4'b1001: next_state = S3;//R type Immediate - ADD/SUB/NAND/OR/Shift
							4'b1010: next_state = S3;
							4'b1101: next_state = S3;
							4'b1110: next_state = S3;
							4'b0111: next_state = S3;
							4'b0110: next_state = S3;
							4'b0000: next_state = S3;
							
							4'b1000: next_state = S4;//R type 2 source - ADD/SUB/NAND/OR
							4'b1100: next_state = S4;
							4'b1011: next_state = S4;
							4'b1111: next_state = S4;
							
							4'b0101: next_state = S5;//BNEQ
							
							4'b0100: next_state = S6;//BEQ
							
							4'b0011: next_state = S7;//JUMP
							default: next_state = 4'bxxxx;
						endcase
					end
				S2: begin
						case(opcode)
							4'b0001: next_state = S8;//Load
							4'b0010: next_state = S9;//store
							default: next_state = 4'bxxxx;
						endcase
					end
				S3: next_state = S10;
				S4: next_state = S10;
				S5: next_state = S0;
				S6: next_state = S0;
				S7: next_state = S0;
				S8: next_state = S11;
				S9: next_state = S0;
				S10: next_state = S0;
				S11: next_state = S0;
				default: next_state = 4'bxx;
			endcase	
		end
endmodule

////////////////////////////////RISC PROCESSOR///////////////////////////////
module multi_cycle(result_is_zero,address_invalid,clk,reset);
	output address_invalid,result_is_zero;
	input clk,reset;
	wire [15:0]pcout,pcin,inst_out,inst_in;
	wire y_or_pcwrite;
	wire [3:0]rb_rs2,ra_rs1,rt_rd;
	wire[1:0]rp,rd;
	wire [7:0]imm8bit;
	wire [11:0]imm12bit;
	wire [3:0]opcode;
	wire [15:0]readdata1,readdata2,readdata3,readdata4,readdata5;
	wire [3:0]readreg1,readreg2,readreg3,readreg4,readreg5,writereg;
	wire [15:0]result,aluin1,aluin2;
	wire zero;
	wire [7:0]operation;
	wire [15:0]address_instmem,address_datamem;
	wire [15:0]data_out,writedata_reg,writedata_mem;
	wire [15:0]out_mdr,in_mdr;
	wire [15:0]out_a,in_a,out_b,in_b,out_c,in_c,out_d,in_d,out_e,in_e,out_aluout,in_aluout;
	wire memwrite,regdst,memtoreg,regwrite,memread,irwrite,instread,
		alucontrolmux,pcwrite,pcwritecond1,pcwritecond2;
	wire[1:0]alusrca,alusrcb,pcsrc;
	wire [3:0]func;
	always@(*)
		$display("PC = %h\n",pcout);
	always@(address_instmem)
		if(alusrcb==2'b11)
			$display("*********New Instruction Begins*********\nIF: PC = PC + 2 \n");
			
	//pc(pcout,pcin,y_or_pcwrite,clk);
	pc pc1(.pcout(pcout),.pcin(pcin),.y_or_pcwrite(y_or_pcwrite),.clk(clk),.reset(reset));
	
	//inst_mem(inst,address,instread);
	inst_mem inst_mem1(.inst(inst_out),.address_invalid(address_invalid),.address(address_instmem),.instread(instread));
	
	//inst_reg(rb_rs2,ra_rs1,rt_rd,rp,rd,imm8bit,imm12bit,opcode,inst,irwrite,clk);
	inst_reg inst_reg1(.rb_rs2(rb_rs2),.ra_rs1(ra_rs1),.rt_rd(rt_rd),
					.rp(rp),.rd(rd),.imm8bit(imm8bit),.imm12bit(imm12bit),
					.opcode(opcode),.inst(inst_in),.irwrite(irwrite),.clk(clk));
					
	//reg_file(readdata1,readdata2,readdata3,readdata4,readdata5,readreg1,readreg2,readreg3,readreg4,readreg5,writereg,writedata,regwrite,clk);
	reg_file register_file1(readdata1,readdata2,readdata3,readdata4,readdata5,
							readreg1,readreg2,readreg3,readreg4,readreg5,
							writereg,writedata_reg,regwrite,clk);
							
	//alu(result,zero,aluin1,aluin2,operation);
	alu alu1(result,zero,aluin1,aluin2,operation);
	
	//data_mem(data,address,writedata,memread,memwrite,clk);
	data_mem data_mem1(data_out,address_datamem,writedata_mem,memread,memwrite,clk);

	//MDR reg_16bits(out,in,clk);
	reg_16bits MDR(out_mdr,in_mdr,clk,reset);
	//A
	reg_16bits A(out_a,in_a,clk,reset);
	//B
	reg_16bits B(out_b,in_b,clk,reset);
	//C
	reg_16bits C(out_c,in_c,clk,reset);
	//D
	reg_16bits D(out_d,in_d,clk,reset);
	//E
	reg_16bits E(out_e,in_e,clk,reset);
	//Aluout
	reg_16bits1 Aluout(out_aluout,in_aluout,clk,reset);
	
	//control_unit
	control_unit main_control(memwrite,regdst,memtoreg,regwrite,memread,irwrite,
							instread,alusrca,alusrcb,alucontrolmux,pcwrite,
							pcwritecond1,pcwritecond2,pcsrc,clk,opcode,reset);

	//Instmem
	assign address_instmem=pcout;

	//IR
	assign inst_in=inst_out;

	//regfile
	assign readreg1=rb_rs2;
	assign readreg2=ra_rs1;
	assign readreg3=rt_rd;
	assign readreg4={2'b10,rp};
	assign readreg5={2'b11,rd};
	assign writereg=regdst?rt_rd:readreg5;
	assign writedata_reg=memtoreg?out_aluout:out_mdr;
	
	//A,B,C,D,E
	assign in_a=readdata1;
	assign in_b=readdata2;
	assign in_c=readdata3;
	assign in_d=readdata4;
	assign in_e=readdata5;
	
	//datamemory
	assign address_datamem=out_aluout;
	assign writedata_mem=out_e;
	
	//MDR
	assign in_mdr=data_out;

	//ALU
	assign aluin1=alusrca[1]?(alusrca[0]?out_d:pcout):(alusrca[0]?out_c:out_a);
	assign aluin2=alusrcb[1]?(alusrcb[0]?16'd2:{{4{imm12bit[11]}},imm12bit}):(alusrcb[0]?{8'd0,imm8bit}:out_b);
	assign operation=alucontrolmux?{func,opcode}:8'bxxxx_1000;
	assign func=rb_rs2;

	//aluout
	assign in_aluout=result;

	//PC
	assign pcin =pcsrc[1]?(pcsrc[0]?2'bxx:result):(pcsrc[0]?out_c:result);
	assign y=(~pcwritecond1)&&pcwritecond2&&(~zero) || pcwritecond1&&(~pcwritecond2)&&zero;
	assign y_or_pcwrite=y||pcwrite;
	
	//Result is zero?
	assign result_is_zero = zero;
endmodule

module waveform();
	//initial 
		//#45 $finish;
		//#1805 $finish;//first posedge at t=5, Total time = #cycles*10 + 5
	initial
		begin
		$dumpfile("multi_cycle.vcd");
		$dumpvars;
		end
endmodule