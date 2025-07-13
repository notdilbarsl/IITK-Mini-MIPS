
`timescale 1ns / 1ps

module CPU_tb;
    reg clk, rst;
    reg [31:0] inst_data;
    reg [9:0] address;
    reg write_instruction, write_data;
    wire [31:0] OutputOfR1 , OutputOfR2 , OutputOfR3 , OutputOfR4 ,OutputOfR5;
    wire done;
    // Instantiate the CPU
    CPU uut (
        .rst(rst),
        .clk(clk),
        .inst_data(inst_data),
        .address(address),
        .write_instruction(write_instruction),
        .write_data(write_data),
        .OutputOfR1(OutputOfR1),
        .OutputOfR2(OutputOfR2),
        .OutputOfR3(OutputOfR3),
        .OutputOfR4(OutputOfR4),
        .OutputOfR5(OutputOfR5),
        .done(done)
    );
    // Clock Generation
    always #5 clk = ~clk;
    initial begin
        // Initialize

  
        //$monitor($time , "Register $6 value = %d\n", uut.RAM.Registers[6]);
    //  $monitor($time , "Reg[0] = %d and PC = %d : Reg[1] = %d , Reg[2] = %d, Reg[3] = %d" , uut.RAM.Registers[0] , uut.PC , uut.RAM.Registers[1], uut.RAM.Registers[2] , uut.RAM.Registers[3]);
      //assign memory_write = (write_data)? inst_data : rt_out;
      //$monitor($time , "PC = %d , memory_write = %d , write_data = %b , inst_data = %d , rt_out = %d , rt = %d" , uut.PC , uut.memory_write , uut.write_data , uut.inst_data , uut.rt_out, uut.rt);
        clk = 0;
        rst = 1;
        address = 0;
        write_instruction = 0;
        write_data = 1; // load with 7,12,9,11,3
        #20
        address = 0;
        inst_data = 7;
        #20
        address = 1;
        inst_data = 12;
        #20
        address = 2;
        inst_data = 9;
        #20
        address = 3;
        inst_data = 11;
        #20
        address = 4;
        inst_data = 3;
        #20
        write_data = 0;
        write_instruction = 1;
        #20;
        // Write the addi instruction to instruction memory at address 0
        address = 0;
        inst_data = 32'b000001_00001_00000_0000_0000_0000_0000;//addi$1,$0,0
        #20;
        address = 1;
        inst_data = 32'b000001_11111_00000_0000_0000_0000_0101; // addi $31 , $0, 5
        #20
        address = 2;
        inst_data = 32'b000001_00001_00000_0000_0000_0000_0000;//addi $1, $0 , 0
        #20
        address = 3;
        inst_data = 32'b000001_00001_00001_0000_0000_0000_0001;//addi $1, $1, 1
        #20
        address = 4;
        inst_data = 32'b010000_00001_11111_0000_0000_0000_1010;//beq $31, $1, 10
        #20
        address = 5;
        inst_data = 32'b000001_00010_00001_1111_1111_1111_1111;//addi $2, $1, -1
        #20
        address = 6;
        inst_data = 32'b000001_00101_00010_0000_0000_0000_0001;//addi $5, $2, 1
        #20
        address = 7;
        inst_data = 32'b000111_00111_00010_0000_0000_0000_0000;//lw $7, $2(0)
        #20
        address = 8;
        inst_data = 32'b000111_01000_00101_0000_0000_0000_0000;//lw $8, $5(0)
        #20
        address = 9;
        inst_data = 32'b010101_01000_00111_1111_1111_1111_1001;//branch to -7 (PC = 3) if $8 >= $7
        #20
        address = 10;
        inst_data = 32'b001000_00111_00101_0000_0000_0000_0000;//sw $7, $5(0)
        #20
        address = 11;
        inst_data = 32'b001000_01000_00010_0000_0000_0000_0000;//sw $8, $2(0)
        #20
        address = 12;
        inst_data = 32'b010000_00010_00000_1111_1111_1111_0110;//beq $2 , $0 , -10 (PC = 3)
        #20
        address = 13;
        inst_data = 32'b000001_00010_00010_1111_1111_1111_1111;//addi $1, $1, 1
        #20
        address = 14;
        inst_data = 32'b010000_00000_00000_1111_1111_1111_0111;//beq $0 , $0, -9 (PC = 6)
      	#20
      	address = 15;
      	inst_data = 32'b000111_00001_00000_0000_0000_0000_0000;//lw$1,$0(0);
      	#20
      	address = 16;
      	inst_data = 32'b000111_00010_00000_0000_0000_0000_0001;//lw$2,$0(1);
      	#20
      	address = 17;
      	inst_data = 32'b000111_00011_00000_0000_0000_0000_0010;//lw$3,$0(2);
      	#20
      	address = 18;
      	inst_data = 32'b000111_00100_00000_0000_0000_0000_0011;//lw$4,$0(3);
      	#20
      	address = 19;
      	inst_data = 32'b000111_00101_00000_0000_0000_0000_0100;//lw$5,$0(4);
      	#20
      	
        rst = 0;
        write_instruction = 0;
        #1000
        // Check output
        $display("Register $31 value = %d", uut.RAM.Registers[31]);
        $display("Register $7 value = %d", uut.RAM.Registers[7]); 
      $display("Register $27 value = %b", uut.RAM.Registers[27]); 
        $display("Register $6 value = %d", uut.RAM.Registers[6]);
        $display("Register $8 value = %d", uut.RAM.Registers[8]);
      $display("Register $10 value = %d", uut.RAM.Registers[10]);
      $display("Register $3 value = %d", uut.RAM.Registers[3]);
      $display("Register $1 value = %d", uut.RAM.Registers[1]);
     $display("Memeroy[0] = %d" , uut.RAM.Registers[1]);
      $display("Memeroy[1] = %d" , uut.RAM.Registers[2]);
      $display("Memeroy[2] = %d" , uut.RAM.Registers[3]);
      $display("Memeroy[3] = %d" , uut.RAM.Registers[4]);
      $display("Memeroy[4] = %d" , uut.RAM.Registers[5]);
      $display("fpr[3] = %b", uut.Fpr.Registers[3]);
      $display("fpr[2] = %b", uut.Fpr.Registers[2]);
      $display("fpr[1] = %b", uut.Fpr.Registers[1]);
      
        $finish;
    end
endmodule
