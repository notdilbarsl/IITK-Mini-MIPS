// Code your design here
// Code your design here
`timescale 1ns / 1ps

module DistributedMemory (
 input wire [9:0] a, input wire[9:0] dpra , input we , input clk ,
 input wire [31 : 0] d , output wire [31 : 0] dpo
);
 reg [31:0] Address_locations [1023:0];
 assign dpo = Address_locations[dpra];
  always @(negedge clk) begin
 if(we) begin
 Address_locations[a] = d;
 end
 else begin
 Address_locations[a] = Address_locations[a];
 end
 end

endmodule
//Reads at positive edge and writes at negative edge
module RegisterFile (
 input wire [4:0] rd , input wire [4:0] rs , input wire [4:0] rt , input clk , input we , input wire [31:0] write_data,
 output reg [31:0] rs_out , output reg [31:0] rt_out, input wire rst , output wire [31:0] Reg1 , output [31:0] Reg2 , 
 output [31:0] Reg3 , output [31:0] Reg4 , output [31:0] Reg5 
);
 reg [31:0] Registers [31:0];
 integer i;
  
  assign Reg1 = Registers[1];
  assign Reg2 = Registers[2];
  assign Reg3 = Registers[3];
  assign Reg4 = Registers[4];
  assign Reg5 = Registers[5];
  
//   assign rs_out = Registers[rs];
//   assign rt_out = Registers[rt];

//I feel better in case of doing something like add $1, $1, $1; 
  always @(rs or rt) begin
    rt_out = Registers[rt];
    rs_out = Registers[rs];
  end
  
 always @(negedge clk ) begin
 if(rst) begin
 for (i = 0; i < 32; i = i + 1) begin
 Registers[i] = 0;
 end
 end
 else begin
 if(we) begin
 Registers[rd] = write_data;
 end
 else begin
 Registers[rd] = Registers[rd];
 end
 end
 end

endmodule




module Splitter (
 input wire [31:0] instruction, output wire [4:0] rt , output wire [4:0] rs , output wire [4:0] rd ,
 output wire [5:0] func , output wire [4:0] shamt , output wire [5:0] opcode , output wire [15:0] address_constant,
 output wire [25:0] jaddress
);
 assign opcode = instruction[31:26];
 assign rd = instruction[25:21];
 assign rt = instruction[20:16];
 assign rs = instruction[15:11];
 assign shamt = instruction[10:6];
 assign func = instruction[5:0];
 assign address_constant = instruction[15:0];
 assign jaddress = instruction[25:0];

endmodule
/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
2 == and
3 == xor
4 == or
5 == not
6 == left shift
7 == right shift
8 == set not equal
9 == set equal
10 == set less than
11 == set less than equal
12 == set greater than
13 == set greater than equal
*/
module ALU (
 input wire [31:0] inp1, input wire [31:0] inp2 , input wire [4:0] ALUctrl , input wire clk ,
 output reg [31:0] ALUout , input wire [4:0] shamt
);
 wire[63:0] product;
 assign product = $signed(inp1) * $signed(inp2);
 always @(inp1 or inp2 or ALUctrl or shamt or product) begin
 case (ALUctrl)
 0: ALUout = inp1 + inp2;
 1: ALUout = inp1 - inp2;
 2: ALUout = inp1 & inp2;
 3: ALUout = inp1 ^ inp2;
 4: ALUout = inp1 | inp2;
 5: ALUout = ~inp1;
 6: ALUout = inp1 << shamt;
 7: ALUout = inp1 >> shamt;
 8: ALUout = ($signed(inp1) != $signed(inp2));
 9: ALUout = ($signed(inp1) == $signed(inp2));
 10: ALUout = ($signed(inp1) < $signed(inp2));
 11: ALUout = ($signed(inp1) <= $signed(inp2));
 12: ALUout = ($signed(inp1) > $signed(inp2));
 13: ALUout = ($signed(inp1) >= $signed(inp2));
 14: ALUout = inp2 << 16;
 15: ALUout = product[31:0];
 16: ALUout = product[63:32];
 default: ALUout = 32'hdeadbeef;
 endcase
 end
endmodule


module Floating_point_to_Binary (
    input [31:0] floating_input , output [31:0] bin_output
);
    assign bin_output = floating_input;    
endmodule


module Binary_to_FloatingPoint (
    input [31:0] bin_input , output [31:0] floating_output
);
    assign floating_output = bin_input;
endmodule

module floating_adder (
    input wire [31:0] inp1 , input [31:0] inp2 , output reg[31:0] out
);
    reg signa , signb;
    reg [7:0] exponenta, exponentb ;
    reg [7:0] diff;
  reg [24:0] ruffa , ruffb;
  reg[24:0] ans;
  reg[24:0] manta, mantb ;
    always @(inp1 or inp2) begin
      
        signa = inp1[31]; signb = inp2[31];
        exponenta = inp1[31:23]; exponentb = inp2[31:23];
      manta = {2'b1, inp1[22:0]}; mantb = {2'b1, inp2[22:0]};
        if(signa == signb) begin
            if(exponenta > exponentb) begin
                diff = exponenta - exponentb;
                ruffb = mantb >> diff;
                ans = ruffb + manta;
              if(ans[24] == 1) begin
                    ans = ans >> 1;
                    exponenta = exponenta + 1;
                end
                else begin
                    ans = ans;
                end
                out = {signa , exponenta , ans[22:0]};
            end
            else begin
                diff = exponentb - exponenta;
                ruffa = manta >> diff;
                ans = ruffa + mantb;
              if(ans[24] == 1) begin
                    ans = ans >> 1;
                    exponentb = exponentb + 1;
                end
                else begin
                    ans = ans;
                end
                out = {signb, exponentb , ans[22:0]};
            end
        end
        else begin
            if(inp1[30:0] > inp2[30:0]) begin
                diff = exponenta - exponentb;
                ruffb = mantb >> diff;
                manta = manta - ruffb;
              if(manta[22] == 1) begin
                exponenta = exponenta - 1;
                manta = manta << 1;
              end 
              else if(manta[21] == 1) begin
                exponenta = exponenta - 2;
                manta = manta << 2;
              end
              else if(manta[20] == 1) begin
                exponenta = exponenta - 3;
                manta = manta << 3;
              end
              else if(manta[19] == 1) begin
                exponenta = exponenta - 4;
                manta = manta << 4;
              end
              else if(manta[18] == 1) begin
                exponenta = exponenta - 5;
                manta = manta << 5;
              end
              else if(manta[17] == 1) begin
                exponenta = exponenta - 6;
                manta = manta << 6;
              end
              else if(manta[16] == 1) begin
                exponenta = exponenta - 7;
                manta = manta << 7;
              end
              else if(manta[15] == 1) begin
                exponenta = exponenta - 8;
                manta = manta << 8;
              end
              else if(manta[14] == 1) begin
                exponenta = exponenta - 9;
                manta = manta << 9;
              end
              else if(manta[13] == 1) begin
                exponenta = exponenta - 10;
                manta = manta << 10;
              end
              else if(manta[12] == 1) begin
                exponenta = exponenta - 11;
                manta = manta << 11;
              end
              else if(manta[11] == 1) begin
                exponenta = exponenta - 12;
                manta = manta << 12;
              end
              else if(manta[10] == 1) begin
                exponenta = exponenta - 13;
                manta = manta << 13;
              end
              else if(manta[9] == 1) begin
                exponenta = exponenta - 14;
                manta = manta << 14;
              end
              else if(manta[8] == 1) begin
                exponenta = exponenta - 15;
                manta = manta << 15;
              end
              else if(manta[7] == 1) begin
                exponenta = exponenta - 16;
                manta = manta << 16;
              end
              else if(manta[6] == 1) begin
                exponenta = exponenta - 17;
                manta = manta << 17;
              end
              else if(manta[5] == 1) begin
                exponenta = exponenta - 18;
                manta = manta << 18;
              end
              else if(manta[4] == 1) begin
                exponenta = exponenta - 19;
                manta = manta << 19;
              end
              else if(manta[3] == 1) begin
                exponenta = exponenta - 20;
                manta = manta << 20;
              end
              else if(manta[2] == 1) begin
                exponenta = exponenta - 21;
                manta = manta << 21;
              end
              else if(manta[1] == 1) begin
                exponenta = exponenta - 22;
                manta = manta << 22;
              end
              else if(manta[0] == 1) begin
                exponenta = exponenta - 23;
                manta = manta << 23;
              end
              else begin
                exponenta = exponenta;
                manta = manta;
              end


              out = {signa , exponenta , manta[22:0]};
            end 
            else begin
                diff = exponentb - exponenta;
                ruffb = manta >> diff;
                manta = mantb - ruffb;
                exponenta = exponentb;
                if(manta[22] == 1) begin
                exponenta = exponenta - 1;
                manta = manta << 1;
              end 
              else if(manta[21] == 1) begin
                exponenta = exponenta - 2;
                manta = manta << 2;
              end
              else if(manta[20] == 1) begin
                exponenta = exponenta - 3;
                manta = manta << 3;
              end
              else if(manta[19] == 1) begin
                exponenta = exponenta - 4;
                manta = manta << 4;
              end
              else if(manta[18] == 1) begin
                exponenta = exponenta - 5;
                manta = manta << 5;
              end
              else if(manta[17] == 1) begin
                exponenta = exponenta - 6;
                manta = manta << 6;
              end
              else if(manta[16] == 1) begin
                exponenta = exponenta - 7;
                manta = manta << 7;
              end
              else if(manta[15] == 1) begin
                exponenta = exponenta - 8;
                manta = manta << 8;
              end
              else if(manta[14] == 1) begin
                exponenta = exponenta - 9;
                manta = manta << 9;
              end
              else if(manta[13] == 1) begin
                exponenta = exponenta - 10;
                manta = manta << 10;
              end
              else if(manta[12] == 1) begin
                exponenta = exponenta - 11;
                manta = manta << 11;
              end
              else if(manta[11] == 1) begin
                exponenta = exponenta - 12;
                manta = manta << 12;
              end
              else if(manta[10] == 1) begin
                exponenta = exponenta - 13;
                manta = manta << 13;
              end
              else if(manta[9] == 1) begin
                exponenta = exponenta - 14;
                manta = manta << 14;
              end
              else if(manta[8] == 1) begin
                exponenta = exponenta - 15;
                manta = manta << 15;
              end
              else if(manta[7] == 1) begin
                exponenta = exponenta - 16;
                manta = manta << 16;
              end
              else if(manta[6] == 1) begin
                exponenta = exponenta - 17;
                manta = manta << 17;
              end
              else if(manta[5] == 1) begin
                exponenta = exponenta - 18;
                manta = manta << 18;
              end
              else if(manta[4] == 1) begin
                exponenta = exponenta - 19;
                manta = manta << 19;
              end
              else if(manta[3] == 1) begin
                exponenta = exponenta - 20;
                manta = manta << 20;
              end
              else if(manta[2] == 1) begin
                exponenta = exponenta - 21;
                manta = manta << 21;
              end
              else if(manta[1] == 1) begin
                exponenta = exponenta - 22;
                manta = manta << 22;
              end
              else if(manta[0] == 1) begin
                exponenta = exponenta - 23;
                manta = manta << 23;
              end
              else begin
                exponenta = exponenta;
                manta = manta;
              end

                out = {signb , exponenta , manta[22:0]};
            end
            //Write the code when both are opposite signs
            
        end
    end
endmodule

module bit_inverser (
    input invert , input [31:0] inp , output [31:0] out
);
    assign out = (invert)? inp ^ 32'h80000000 : inp;
endmodule

//Put rt into inp1
module FPU (
    input wire [31:0] inp1 , input wire [31:0] inp2 , output reg [31:0] FPUout,
    input wire [5:0] opcode
);
    wire [31:0] Adder_cout;
    reg cc;
    wire invert;
    assign invert = (opcode == 6'b100011);
    wire [31:0] inp2_intoALU;
    bit_inverser bt(.invert(invert) , .inp(inp2) , .out(inp2_intoALU));
    floating_adder fa(.inp1(inp1) , .inp2(inp2_intoALU) , .out(Adder_cout));
    always @(inp1 or inp2 or opcode or Adder_cout) begin
        case (opcode)
            6'b100010: FPUout = Adder_cout;//add.s
            6'b100011: FPUout = Adder_cout;//sub.s
            6'b100100: cc = (inp1 == inp2);//c.eq.ss
            6'b100101: cc = (inp1 <= inp2);//c.le.ss
            6'b100110: cc = (inp1 < inp2);//c.lt.ss
            6'b100111: cc = (inp1 >= inp2);//c.ge.ss
            6'b101000: cc = (inp1 > inp2);//c.gt.ss
            6'b101001: FPUout = inp1;//mov.s.cc
            default: FPUout = FPUout;
        endcase
    end
endmodule

module PC_Controller (
 input clk , output reg [9:0] PC, input rst , input jump , input [25:0] jaddress , input branch , input [15:0] branchval
);
 always @(posedge clk ) begin
 if(rst) begin
 PC = 0;
 end
 else begin
 if(jump) begin
 PC = jaddress;
 end
 else if(branch) begin
 PC = PC + 1 + branchval;
 end
 else if(PC <= 100) begin
 PC = PC + 1;
 end
 else begin
  PC = PC;
 end
 end
 end
endmodule

/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
2 == and
3 == xor
4 == or
5 == not
6 == left shift
7 == right shift
8 == set not equal
9 == set equal
10 == set less than
11 == set less than equal
12 == set greater than
13 == set greater than equal
*/
module ALU_controller (
 input wire [5:0] opcode , input wire[5:0] func , output reg [4:0] ALUctrl
);
 always @(opcode or func) begin
 if(opcode == 6'b000111 || opcode == 6'b001000) begin // lw or sw
 ALUctrl = 0; // add
 end
 else if(opcode == 1) begin // addi
 ALUctrl = 0;
 end
 else if(opcode == 2) begin // andi
 ALUctrl = 2;
 end
 else if(opcode == 3) begin // ori
 ALUctrl = 4;
 end
 else if(opcode == 4) begin // xori
 ALUctrl = 3;
 end
 else if(opcode == 5) begin // addui
 ALUctrl = 0;
 end
 else if(opcode == 6'b001001 || opcode == 6'b010100 || opcode == 6'b010110) begin // slti and ble , bleu
 ALUctrl = 10;
 end
 else if(opcode == 6'b001010 || opcode == 6'b010000) begin // seq, beq
 ALUctrl = 9;
 end
 else if(opcode == 6'b010001) begin // bne
 ALUctrl = 8;
 end
 else if(opcode == 6'b010010 || opcode == 6'b010111) begin // bgt ,bgtu
 ALUctrl = 12;
 end
 else if(opcode == 6'b010011) begin // bgte
 ALUctrl = 13;
 end
 else if(opcode == 6'b010101) begin // bleq
 ALUctrl = 11;
 end
 else if(opcode == 6'b001011) begin//lui
 ALUctrl = 14;
 end
 else begin
 if(opcode == 6'b000000) begin
 case (func)
 0: ALUctrl = 0;
 1: ALUctrl = 1;
 2: ALUctrl = 2;
 3: ALUctrl = 4;
 4: ALUctrl = 5;
 5: ALUctrl = 3;
 6: ALUctrl = 0;
 7: ALUctrl = 1;
 8: ALUctrl = 10;
 9: ALUctrl = 6;
 10: ALUctrl = 7;
 11: ALUctrl = 16;
 12: ALUctrl = 15;
 default: ALUctrl = 31;
 endcase
 end
 else begin
 ALUctrl = 5'b11111;
 end
 end
 end
endmodule

module SignExtender (
 input wire[15:0] inp , output wire [31:0] out
);
 assign out = {{16{inp[15]}}, inp};
endmodule

module mux2_1 #(
 parameter size = 32
) (
 input wire [size - 1 : 0] inp0 , input wire [size - 1 : 0] inp1, input select , output wire[size - 1 : 0] out
);
 assign out = (select)? inp0 : inp1;
endmodule



module Controller (
 input clk , input wire [5:0] opcode , output reg write_reg , output reg data_write ,
 output reg immediate , output reg jump, output reg branch , output reg jal , output reg select_ALU_or_Mem,//0 for ALU, 1 for MEM
 input rst , output reg mfc1
);
 always @(opcode or rst) begin
 if(rst) begin
 immediate = 0;
 jump = 0;
 branch = 0;
 select_ALU_or_Mem = 0;
 data_write = 0;
 write_reg = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 0) begin // Rtype instruction
 data_write = 0;
 write_reg = 1;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 1 ||opcode == 2 ||opcode == 3 ||opcode == 4 ||opcode == 5) begin //addi, andi , xori, ori , addiu
 data_write = 0;
 write_reg = 1;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 7) begin // lw
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 1;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 8) begin // sw
 write_reg = 0;
 data_write = 1;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 9 || opcode == 10) begin // slti , seq
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 6'b001011) begin //lui
 write_reg = 1;
 data_write = 0;
 immediate = 1;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 16 || opcode == 17 ||opcode == 18 || opcode == 19 || opcode == 20 || opcode == 21 || opcode == 22 || opcode == 23) begin //all branches
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 1;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 6'b011000 || opcode == 6'b011001) begin // All jump except jal
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 1;
 jal = 0;
 mfc1 = 0;
 end
 else if(opcode == 6'b011010) begin // jal handled seperately as it requires to store the value of PC + 4 into $ra
 write_reg = 1;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 1;
 jal = 1;
 mfc1 = 0;
 end
 else if(opcode == 6'b100000) begin//mfc1
 write_reg = 1;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 1;
 end
 else begin
 write_reg = 0;
 data_write = 0;
 immediate = 0;
 select_ALU_or_Mem = 0;
 branch = 0;
 jump = 0;
 jal = 0;
 mfc1 = 0;
 end
 end
endmodule



module FPR_controller (
    input [5:0] opcode, output write_fpr , output mtc1
);
    assign write_fpr = (opcode == 6'b100001 || opcode == 6'b100010 || opcode == 6'b100011 || opcode == 6'b101001);//mtc1, add.s, sub.s , mov.s.cc
    assign mtc1 = (opcode == 6'b100001);
endmodule

module CPU (
 input rst , input clk , input wire [31:0] inst_data ,
 input wire [9:0] address , input wire write_instruction , input wire write_data,
 output wire [31:0] OutputOfR1 , output wire [31:0] OutputOfR2, output wire [31:0] OutputOfR3,
 output wire [31:0] OutputOfR4, output wire [31:0] OutputOfR5 , output wire done
);
// .a(a), // input wire [8 : 0] a. This is the write Address
// .d(d), // input wire [31 : 0] d. This is the data to be written
// .dpra(dpra), // input wire [8 : 0] dpra. This is the read Address
// .clk(clk), // input wire clk
// .we(we), // input wire we
// .dpo(dpo) // output wire [31 : 0] dpo. the value that is read
 
 wire [9:0] PC;
 wire [31:0] instruction;
 wire [31:0] ALU_out;
 wire [9:0] memory_in;
 wire [31:0] rt_out , rs_out , memory_write , memory_out , rs_or_address_to_ALU, extended_address , rtout_f, rsout_f , FPU_out , converted_bin , converted_float , final_wire_going_into_register_rd , data_write_in_fpr;
 wire [4:0] rt , rs , rd , shamt;
 wire [5:0] opcode , func;
 wire [15:0] address_constant;
 wire [25:0] jaddress;
 wire branch , jump , jal , mem_write , immediate, select_ALU_or_Mem, write_reg, write_f_register , mtc1 , mfc1;
 wire [4:0] wire_going_into_rs;
 //mem_write instead of data_write
 wire [4:0] ALUctrl;
 assign wire_going_into_rs = (branch | mem_write)? rd : rs;
 assign done = (PC >= 50);
 //this thing is to handle jal
 wire[4:0] write_in_Register;
 assign write_in_Register = (jal)? 5'd3 : rd;
 wire [31:0] write_data_in_register , wire_going_into_register_rd;
 assign wire_going_into_register_rd = (jal)? {22'b0 , PC} : write_data_in_register;
 assign final_wire_going_into_register_rd = (mfc1)? converted_bin : wire_going_into_register_rd; // essentially for mfc1
 //write_data_in_register is coming from the last mux(selecting ALU or memory Output)
 //wire_going_into_register_rd is write data that is actually going into register file
 //write_in_register goes into rd
 // mux2_1 rt_ALU (rt_out , extended_address , immediate , rs_or_address_to_ALU);


 assign rs_or_address_to_ALU = (immediate)? extended_address : rs_out;
 SignExtender sign_extend_addr_const(.inp(address_constant) , .out(extended_address));
 // mux2_1 Last_mem_stage(ALU_out , memory_out , select_ALU_or_Mem , write_data_in_register);
 assign write_data_in_register = (select_ALU_or_Mem)? memory_out : ALU_out;
 dist_mem_gen_1 instruction_mem(.a(address),.d(inst_data),.dpra(PC),.clk(clk),.we(write_instruction),.dpo(instruction));

// mux2_1 #(.size(10)) ALU_Mem_write_address (ALU_out[9:0] , address , write_data , memory_in);
 assign memory_in = (write_data)? address : ALU_out[9:0];
//  mux2_1 What_to_write_decider(rt_out , inst_data , write_data , memory_write);
 assign memory_write = (write_data)? inst_data : rs_out;
 dist_mem_gen_1 data_mem(.a(memory_in) , .d(memory_write) , .dpra(ALU_out[9:0]) , .clk(clk) , .we(mem_write | write_data) , .dpo(memory_out));

 Splitter split(.instruction(instruction) , .rt(rt) , .rs(rs) , .rd(rd) , .shamt(shamt), .func(func), .opcode(opcode) , .address_constant(address_constant) , .jaddress(jaddress));
 Controller brain(.clk(clk) , .opcode(opcode) , .write_reg(write_reg) , .data_write(mem_write) ,
 .immediate(immediate) , .jump(jump) , .branch(branch) , .jal(jal) , .select_ALU_or_Mem(select_ALU_or_Mem) ,
 .rst(rst) , .mfc1(mfc1));
 RegisterFile RAM(.rd(write_in_Register) , .rs(wire_going_into_rs) , .rt(rt) , .clk(clk) , .we(write_reg) , .write_data(final_wire_going_into_register_rd) , .rst(rst) , .rt_out(rt_out) , .rs_out(rs_out) ,
                   .Reg1(OutputOfR1) , .Reg2(OutputOfR2) , .Reg3(OutputOfR3) , .Reg4(OutputOfR4) , .Reg5(OutputOfR5)    );

 ALU_controller nerves(.opcode(opcode) , .func(func) , .ALUctrl(ALUctrl));
 ALU brawn(.inp1(rt_out) , .inp2(rs_or_address_to_ALU) , .ALUctrl(ALUctrl) , .clk(clk) , .ALUout(ALU_out) , .shamt(shamt));
 PC_Controller legs(.clk(clk) , .PC(PC) , .rst(rst) , .jump(jump) , .jaddress(jaddress) , .branch(branch & ALU_out[0]) , .branchval(address_constant));


    //FP registers
    assign data_write_in_fpr = (mtc1)? converted_float: FPU_out;
    Floating_point_to_Binary fptb(.floating_input(rtout_f) , .bin_output(converted_bin));
    Binary_to_FloatingPoint btfp(.bin_input(rt_out) , .floating_output(converted_float));


    RegisterFile Fpr(.rd(rd) , .rs(rs) , .rt(rt) , .clk(clk) , .we(write_f_register) , .write_data(data_write_in_fpr) , .rs_out(rsout_f) , .rt_out(rtout_f) , .rst(rst));

    FPR_controller fpr_ctrlr(.opcode(opcode) , .write_fpr(write_f_register) , .mtc1(mtc1));

    FPU floating_point_ALU(.inp1(rsout_f) , .inp2(rtout_f) , .FPUout(FPU_out) , .opcode(opcode));

endmodule
