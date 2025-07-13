`timescale 1ns/1ps

module Binary_to_FloatingPoint_tb;

    reg  [31:0] bin_input;
    wire [31:0] floating_output;

    // Instantiate the DUT (device under test)
    Binary_to_FloatingPoint dut (
        .bin_input(bin_input),
        .floating_output(floating_output)
    );

    initial begin
        $display("==== BINARY TO FLOATING POINT CONVERTER TEST ====");
		#100
        // Case 1: bin_input = 5 (0x00000005)
        bin_input = 32'd5;
      #10
      $display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
         $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);
		$display("-------------------------------------------");
        // Case 2: bin_input = 2 (0x00000002)
        bin_input = 32'd2;
      #10
      $display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
        $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);
      $display("-------------------------------------------");
        // Case 3: bin_input = 1 (0x00000001)
        bin_input = 32'd1;
      #10
      $display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
         $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);
      $display("-------------------------------------------");
      
        // Case 4: bin_input = 0
        bin_input = 32'd0;
      	#10
      	$display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
        $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);
		$display("-------------------------------------------");
      
        // Case 5: bin_input = 100
        bin_input = 32'd6;
      #10
      	$display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
         $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);
      $display("-------------------------------------------");
      
      //case6 : bin_input = 124
      bin_input = 32'd124;
      #10
		$display("mantissa = %b" , dut.mantissa);
      $display("exponent = %b" , dut.exponent - 8'd127);
         $display("bin_input = %b (%0d), floating_output = %h", bin_input, bin_input, floating_output);      

        $display("==== TEST COMPLETE. FLOAT EM IF YOU GOT EM 😎 ====");
        $finish;
    end

endmodule
