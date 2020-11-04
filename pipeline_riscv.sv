// FETCH -------------------------------------------------------------------------------
module fetch (input zero, rst, clk, branch, input [31:0] sigext, output [31:0] d_inst, d_pc);
  
  wire [31:0] pc, pc_4, new_pc, inst;

  assign pc_4 = 4 + pc; // pc+4  Adder
  assign new_pc = (branch & zero) ? pc_4 + sigext : pc_4; // new PC Mux

  PC program_counter(new_pc, clk, rst, pc);

  reg [31:0] inst_mem [0:31];

  assign inst = inst_mem[pc[31:2]];
  
  // PIPE F -> D
  IFID IFID (clk, rst, pc_4, inst, d_pc, d_inst);

  initial begin
    // Exemplos
    /*inst_mem[0] <= 32'h00000000; // nop
    inst_mem[1] <= 32'h00500113; // addi x2, x0, 5  ok  valor 5
    inst_mem[2] <= 32'h00210233; // add  x4, x2, x2  ok  valor 4
    inst_mem[3] <= 32'h00202423; // sw x2, 8(x0) ok  valor 8
    //inst_mem[4] <= 32'h0050a423; // sw x5, 8(x1) ok 
    inst_mem[4] <= 32'h00802083; // lw x1, 8(x0) ok  valor 2
    inst_mem[5] <= 32'h00208233; // add x4, x1, x2 ok valor 6
    // inst_mem[6] <= 32'h01002083; // lw x1, 16(x0) ok
    //inst_mem[2] <= 32'h0000a003; // lw x1, x0(0) ok
    //inst_mem[1] <= 32'hfff00113; // addi x2,x0,-1 ok
    //inst_mem[2] <= 32'h00318133; // add x2, x3, x3 ok
    //inst_mem[3] <= 32'h40328133; // sub x2, x5, x3 ok*/
    
    inst_mem[0] <= 32'h00318133; // add x2, x3, x3 6
    inst_mem[1] <= 32'h00328233; // add x4, x5, x3 8
    inst_mem[2] <= 32'h00802083; // lw x1, 8(x0) 2
    inst_mem[3] <= 32'h00402183; // lw x3, 4(x0) 1
    inst_mem[4] <= 32'h00202823; // sw x2, 16(x0) 16
  end
  
endmodule

module PC (input [31:0] pc_in, input clk, rst, output reg [31:0] pc_out);

  always @(posedge clk) begin
    pc_out <= pc_in;
    if (~rst)
      pc_out <= 0;
  end

endmodule

module IFID (input f_clk, f_rst, input [31:0] f_pc, f_inst,
             output reg [31:0] d_pc, d_inst);
  always @(posedge f_clk) begin
    if (~f_rst) begin
      d_inst <= 0;
      d_pc <= 0;
    end
    else begin
      d_inst <= f_inst;
      d_pc <= f_pc;
    end
  end
endmodule

// END FETCH ----------------------------------------------------------------------------

// DECODE ----------------------------------------------------------------------------
module decode (input [31:0] inst, writedata, pc,
               input clk, rst, regwrite,
               input [4:0] regdst,
               output reg e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_alusrc, 
               output reg [1:0] e_aluop, 
               output reg [31:0] e_pc, e_data1, e_data2, e_ImmGen, 
               output reg [4:0] e_rs1, e_rs2, e_regdst,
               output reg [9:0] e_funct);
  
  wire branch, memread, memtoreg, memwrite, alusrc;
  wire [1:0] aluop; 
  wire [4:0] writereg, rs1, rs2, rd;
  wire [6:0] opcode;
  wire [9:0] funct;
  wire [31:0] data1, data2, ImmGen;

  assign opcode = inst[6:0];
  assign rs1    = inst[19:15];
  assign rs2    = inst[24:20];
  assign rd     = inst[11:7];
  assign funct = {inst[31:25],inst[14:12]};

  ControlUnit control (opcode, inst, alusrc, memtoreg, regwrite_out, memread, memwrite, branch, aluop, ImmGen);
  
  Register_Bank Registers (clk, regwrite, rs1, rs2, regdst, writedata, data1, data2);
  
  IDEX IDEX (clk, rst, regwrite_out, memtoreg, branch, memwrite, memread, alusrc,
             aluop, pc, data1, data2, ImmGen, rs1, rs2, rd, funct, e_regwrite, e_memtoreg, e_branch, e_memwrite, 
             e_memread, e_alusrc, e_aluop, e_pc, e_data1, e_data2, e_ImmGen, e_rs1, e_rs2, e_regdst, e_funct);

endmodule

// pipe2 D -> E
module IDEX (input d_clk, rst, d_regwrite, d_memtoreg, d_branch, d_memwrite, d_memread, d_alusrc,
             input [1:0] d_aluop,
             input [31:0] d_pc, d_rd1, d_rd2, d_ImmGen,
             input [4:0] d_rs1, d_rs2, d_regdst,
             input [9:0] d_funct,
             output reg e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_alusrc,
             output reg [1:0] e_aluop,
             output reg [31:0] e_pc, e_rd1, e_rd2, e_ImmGen,
             output reg [4:0] e_rs1, e_rs2, e_regdst,
             output reg [9:0] e_funct);
  always @(posedge d_clk) begin
    if (!rst) begin
      e_regwrite <= 0;
      e_memtoreg <= 0;
      e_branch   <= 0;
      e_memwrite <= 0;
      e_memread  <= 0;
      e_regdst   <= 0;
      e_aluop    <= 0;
      e_alusrc   <= 0;
      e_pc       <= 0;
      e_rd1      <= 0;
      e_rd2      <= 0;
      e_ImmGen   <= 0;
      e_rs1    <= 0;
      e_rs2    <= 0;
      e_funct <= 0;
    end
    else begin
      e_regwrite <= d_regwrite;
      e_memtoreg <= d_memtoreg;
      e_branch   <= d_branch;
      e_memwrite <= d_memwrite;
      e_memread  <= d_memread;
      e_regdst   <= d_regdst;
      e_aluop    <= d_aluop;
      e_alusrc   <= d_alusrc;
      e_pc       <= d_pc;
      e_rd1      <= d_rd1;
      e_rd2      <= d_rd2;
      e_ImmGen   <= d_ImmGen;
      e_rs1    <= d_rs1;
      e_rs2    <= d_rs2;
      e_funct <= d_funct;
    end
  end
endmodule

module ControlUnit (input [6:0] opcode, input [31:0] inst, output reg alusrc, memtoreg, regwrite, memread, memwrite, branch, output reg [1:0] aluop, output reg [31:0] ImmGen);

  always @(opcode) begin
    alusrc   <= 0;
    memtoreg <= 0;
    regwrite <= 0;
    memread  <= 0;
    memwrite <= 0;
    branch   <= 0;
    aluop    <= 0;
    ImmGen   <= 0; 
    case(opcode) 
      7'b0110011: begin // R type == 51
        regwrite <= 1;
        aluop    <= 2;
			end
		  7'b1100011: begin // beq == 99
        branch   <= 1;
        aluop    <= 1;
        ImmGen   <= {{19{inst[31]}},inst[31],inst[7],inst[30:25],inst[11:8],1'b0};
			end
			7'b0010011: begin // addi == 19
        alusrc   <= 1;
        regwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
			7'b0000011: begin // lw == 3
        alusrc   <= 1;
        memtoreg <= 1;
        regwrite <= 1;
        memread  <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:20]};
      end
			7'b0100011: begin // sw == 35
        alusrc   <= 1;
        memwrite <= 1;
        ImmGen   <= {{20{inst[31]}},inst[31:25],inst[11:7]};
      end
    endcase
  end

endmodule

module Register_Bank (input clk, regwrite, input [4:0] read_reg1, read_reg2, writereg, input [31:0] writedata, output [31:0] read_data1, read_data2);

  integer i;
  reg [31:0] memory [0:31]; // 32 registers de 32 bits cada

  // fill the memory
  initial begin
    for (i = 0; i <= 31; i++) 
      memory[i] <= i;
  end

  assign read_data1 = (regwrite && read_reg1==writereg) ? writedata : memory[read_reg1];
  assign read_data2 = (regwrite && read_reg2==writereg) ? writedata : memory[read_reg2];
	
  always @(posedge clk) begin
    if (regwrite)
      memory[writereg] <= writedata;
  end
  
endmodule

// END DECODE ----------------------------------------------------------------------------

// EXECUTE -------------------------------------------------------------------------------
module execute (input e_clk, rst, e_regwrite, e_memtoreg, e_branch, e_memread,
                e_memwrite, e_alusrc,
                input [1:0] e_aluop,
                input [31:0] e_pc, e_data1, e_data2, e_ImmGen,
                input [4:0] e_rs1, e_rs2, e_regdst,
                input [9:0] funct,
                output reg m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch,
                output reg [4:0] m_regdst,
                output reg [31:0] m_aluout, m_data2, m_addres);

  wire [31:0] alu_B, e_addres, e_aluout;
  wire [3:0] aluctrl;
  wire e_zero;
  
  Add Add (e_pc, e_sigext, e_addres);
  
  assign alu_B = (e_alusrc) ? e_ImmGen : e_data2;

  //Unidade Lógico Aritimética
  ALU alu (aluctrl, e_data1, alu_B, e_aluout, e_zero);

  alucontrol alucontrol (e_aluop, funct, aluctrl);
  
  // PIPE E -> M
  EXMEM EXMEM (e_clk, rst, e_regwrite, e_memtoreg, e_branch, e_zero, e_memread, e_memwrite, e_regdst, e_addres, e_aluout, e_data2,
               m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch, m_regdst, m_addres, m_aluout, m_data2);

endmodule

module Add (input [31:0] pc, shiftleft2, output [31:0] add_result);
  assign add_result = pc + (shiftleft2 << 2);
endmodule

// pipe2 E -> M
module EXMEM (input e_clk, rst, e_regwrite, e_memtoreg, e_branch, e_zero, e_memread, e_memwrite,
              input [4:0] e_regdst,
              input [31:0] e_addres, e_aluout, e_data2,
              output reg m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch,
              output reg [4:0] m_regdst,
              output reg [31:0] m_addres, m_aluout, m_data2);
  always @(posedge e_clk) begin
    if (!rst) begin
      m_regwrite  <= 0;
      m_memtoreg  <= 0;
      m_addres    <= 0;
      m_zero      <= 0;
      m_regdst 	  <= 0;
      m_aluout    <= 0;
      m_data2     <= 0;
      m_memread   <= 0;
      m_memwrite  <= 0;
      m_branch    <= 0;
    end
    else begin
      m_regwrite  <= e_regwrite;
      m_memtoreg  <= e_memtoreg;
      m_addres    <= e_addres;
      m_zero      <= e_zero;
      m_regdst 	  <= e_regdst;
      m_aluout    <= e_aluout;
      m_data2     <= e_data2;
      m_memread   <= e_memread;
      m_memwrite  <= e_memwrite;
      m_branch    <= e_branch;
    end
  end
endmodule

module alucontrol (input [1:0] aluop, input [9:0] funct, output reg [3:0] alucontrol);
  
  wire [7:0] funct7;
  wire [2:0] funct3;

  assign funct3 = funct[2:0];
  assign funct7 = funct[9:3];

  always @(aluop) begin
    case (aluop)
      0: alucontrol <= 4'd2; // ADD to SW and LW
      1: alucontrol <= 4'd6; // SUB to branch
      default: begin
        case (funct3)
          0: alucontrol <= (funct7 == 0) ? /*ADD*/ 4'd2 : /*SUB*/ 4'd6; 
          2: alucontrol <= 4'd7; // SLT
          6: alucontrol <= 4'd1; // OR
          //39: alucontrol <= 4'd12; // NOR
          7: alucontrol <= 4'd0; // AND
          default: alucontrol <= 4'd15; // Nop
        endcase
      end
    endcase
  end
endmodule

module ALU (input [3:0] alucontrol, input [31:0] A, B, output reg [31:0] aluout, output zero);
  
  assign zero = (aluout == 0); // Zero recebe um valor lógico caso aluout seja igual a zero.
  
  always @(alucontrol, A, B) begin
      case (alucontrol)
        0: aluout <= A & B; // AND
        1: aluout <= A | B; // OR
        2: aluout <= A + B; // ADD
        6: aluout <= A - B; // SUB
        7: aluout <= A < B ? 32'd1:32'd0; //SLT
        // 12: aluout <= ~(A | B); // NOR
      default: aluout <= 0; //default 0, Nada acontece;
    endcase
  end
endmodule

// END EXECUTE ----------------------------------------------------------------------------

// MEMORY ---------------------------------------------------------------------------------

module memory (input [31:0] address, writedata,
               input [4:0] m_regdst,
               input memread, memwrite, clk, rst, m_regwrite, m_memtoreg,
               output reg [31:0] w_readdata, w_aluout, 
               output reg [4:0] w_regdst,
               output reg w_regwrite, w_memtoreg);

  integer i;
  reg [31:0] memory [0:127];
  wire [31:0] m_readdata;
  
  // fill the memory
  initial begin
    for (i = 0; i <= 127; i++) 
      memory[i] <= i;
  end

  assign m_readdata = (memread) ? memory[address[31:2]] : 0;

  always @(posedge clk) begin
    if (memwrite)
      memory[address[31:2]] <= writedata;
	end
  
  MEMWB MEMWB (clk, rst, m_regwrite, m_memtoreg, m_readdata, address, m_regdst,
               w_readdata, w_aluout, w_regdst, w_regwrite, w_memtoreg);
endmodule

// pipe3 M -> W
module MEMWB (input m_clk, rst, m_regwrite, m_memtoreg,
              input [31:0] m_readData, m_aluout,
              input [4:0] m_regdst,
              output reg [31:0] w_readData, w_aluout,
              output reg [4:0] w_regdst,
              output reg w_regwrite, w_memtoreg);
  always @(posedge m_clk) begin
    if (!rst) begin
      w_readData  <= 0;
      w_aluout    <= 0;
      w_regwrite  <= 0;
      w_memtoreg  <= 0;
      w_regdst 	  <= 0;
    end
    else begin
      w_readData  <= m_readData;
      w_aluout    <= m_aluout;
      w_regwrite  <= m_regwrite;
      w_memtoreg  <= m_memtoreg;
      w_regdst 	  <= m_regdst;
    end
  end
endmodule

// END MEMORY -----------------------------------------------------------------------------

// WRITEBACK ------------------------------------------------------------------------------

module writeback (input [31:0] aluout, readdata,
                  input memtoreg,
                  output reg [31:0] write_data);
    assign write_data = (memtoreg) ? readdata : aluout;
endmodule

// END WRITEBACK --------------------------------------------------------------------------

// TOP -------------------------------------------
// module mips (input clk, rst, output [31:0] writedata);
module mips (input clk, rst, output [31:0] writedata);
  
  wire [31:0] d_inst, d_pc, e_pc, e_data1, e_data2, e_ImmGen, writedata, m_data2, m_addres, add_res, m_aluout, m_readdata, w_readdata, w_aluout, reg_writedata;
  wire e_regwrite, e_memtoreg, e_branch, e_memwrite, e_memread, e_alusrc, m_regWrite, m_memtoreg, m_zero, m_memread, m_memwrite, w_regwrite, w_memtoreg, m_branch, branch, zero;
  wire [1:0] e_aluop;
  wire [4:0] e_rs1, e_rs2, e_muxRegDst, e_regdst, m_regdst, w_regdst;
  wire [9:0] funct;
  
  // FETCH STAGE
  fetch fetch (m_zero, rst, clk, m_branch, m_addres, d_inst, d_pc);
  
  // DECODE STAGE
  decode decode (d_inst, writedata, d_pc, clk, rst, w_regwrite, w_regdst, e_regwrite, e_memtoreg, e_branch,
                 e_memwrite, e_memread, e_alusrc, e_aluop, e_pc, e_data1, e_data2,
                 e_ImmGen, e_rs1, e_rs2, e_regdst, funct);
  
  // EXECUTE STAGE
  execute execute (clk, rst, e_regwrite, e_memtoreg, e_branch, e_memread, e_memwrite, e_alusrc,
                   e_aluop, e_pc, e_data1, e_data2, e_ImmGen, e_rs1, e_rs2, e_regdst, funct,
                   m_regwrite, m_memtoreg, m_zero, m_memread, m_memwrite, m_branch, m_regdst, m_aluout,
                   m_data2, m_addres);

  // MEMORY STAGE
  memory memory (m_aluout, m_data2, m_regdst, m_memread, m_memwrite, clk, rst,
                 m_regwrite, m_memtoreg, w_readdata, w_aluout, w_regdst, w_regwrite, w_memtoreg);
  
  // WRITEBACK STAGE
  writeback writeback (w_aluout, w_readdata, w_memtoreg, writedata);
  
endmodule
