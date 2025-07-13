
`timescale 1ns / 1ps

module CPU_tb;
    reg clk, rst;
    reg [31:0] inst_data;
    reg [9:0] address;
    reg write_instruction, write_data;
    wire [31:0] OutputOfR1 , OutputOfR2 , OutputOfR3 , OutputOfR4 , OutputOfR5;
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
     // $monitor($time , "Reg[0] = %d and PC = %d : Reg[1] = %d , Reg[2] = %d, Reg[3] = %d" , uut.RAM.Registers[0] , uut.PC , uut.RAM.Registers[1], uut.RAM.Registers[2] , uut.RAM.Registers[3]);
      //assign memory_write = (write_data)? inst_data : rt_out;
      //$monitor($time , "PC = %d , memory_write = %d , write_data = %b , inst_data = %d , rt_out = %d , rt = %d" , uut.PC , uut.memory_write , uut.write_data , uut.inst_data , uut.rt_out, uut.rt);
        clk = 0;
        rst = 1;
        address = 0;
        write_instruction = 1;
        write_data = 0; // load with 7,12,9,11,3
        #20;
        // Write the addi instruction to instruction memory at address 0
        address = 0;
        inst_data = 32'b000001_00001_00000_0000_0000_0000_0000;//addi$1,$0,0
        #20;
        address = 1;
        inst_data = 32'b001011_11111_00000_0100000000100010; //basically loading $31 with 2.52
        #20
        address = 2;
        inst_data = 32'b000001_11111_11111_1000111101011100;// Basically loading $31 with 2.52
        #20
        address = 3;
        inst_data = 32'b001011_11110_00000_0100000110000011;//basically loading $30 with 16.48
        #20
        address = 4;
        inst_data = 32'b000001_11110_11110_1101011100001010;//basically loading $30 with 16.48
        #20
        address = 5;
        inst_data = 32'b100001_00010_11110_0000_0000_0000_0000;//mtc1 $2 , $30;
        #20
        address = 6;
        inst_data = 32'b100001_00001_11111_0000_0000_0000_0000;//mtc1 $1, $31;
        #20
        address = 7;
        inst_data = 32'b100010_00011_00010_00001_00000_000000;//add.s$3,$2,$1;
      	#20
      	address = 8;
      	inst_data = 32'b100011_00100_00001_00010_00000_000000;//sub.s $4,$2,$1
        #20
      	address = 9;
      	inst_data = 32'b100000_00001_00011_0000_0000_0000_0000;//mfc1 $1,$3;
      	#20
      	address = 10;
      	inst_data = 32'b000001_00010_00000_0000_0000_0000_1010;//addi$2,$0,2
      	#20
      	address = 11;
      	inst_data = 32'b000001_00011_00000_0000_0000_0000_1000;//addi$3,$0,8;
      	#20
      	address = 12;
      	inst_data = 32'b000000_00100_00010_00011_00000_001100;//mflo$4,$2,$3;
      	#20
        rst = 0;
        write_instruction = 0;
        #200
        // Check output
        $display("Register $31 value = %b", uut.RAM.Registers[31]);
      $display("Register $30 value = %b", uut.RAM.Registers[30]); 
      $display("Register $1 value = %b", uut.RAM.Registers[1]);
      $display("Register $2 value = %d", uut.RAM.Registers[2]);
      $display("Register $3 value = %d", uut.RAM.Registers[3]);
      $display("Register $4 value = %d", uut.RAM.Registers[4]);
    //  $display("Memeroy[0] = %d" , uut.data_mem.Address_locations[0]);
    //   $display("Memeroy[1] = %d" , uut.data_mem.Address_locations[1]);
    //   $display("Memeroy[2] = %d" , uut.data_mem.Address_locations[2]);
    //   $display("Memeroy[3] = %d" , uut.data_mem.Address_locations[3]);
    //   $display("Memeroy[4] = %d" , uut.data_mem.Address_locations[4]);
      $display("fpr[4] = %b", uut.Fpr.Registers[4]);
      $display("fpr[3] = (2.52 + 16.33) = %b", uut.Fpr.Registers[3]);
      $display("fpr[2] = %b", uut.Fpr.Registers[2]);
      $display("fpr[1] = %b", uut.Fpr.Registers[1]);
      
        $finish;
    end
endmodule
