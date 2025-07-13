// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

module floating_adder_tb;

    // Inputs
    reg [31:0] inp1, inp2;

    // Output
    wire [31:0] out;

    // Instantiate the Unit Under Test (UUT)
    floating_adder uut (
        .inp1(inp1),
        .inp2(inp2),
        .out(out)
    );

    // Task to print inputs and outputs as real numbers
    task print_result;
        input [31:0] a, b, res;
        real ra, rb, rres;
        begin
            ra = $bitstoreal({32'b0, a});
            rb = $bitstoreal({32'b0, b});
            rres = $bitstoreal({32'b0, res});
            $display("A = %f, B = %f => Result = %f", ra, rb, rres);
        end
    endtask

    initial begin
        $display("Testing floating point adder...");

        // Test 1: 1.5 + 2.25 = 3.75
        inp1 = 32'h3FC00000; // 1.5
        inp2 = 32'h40200000; // 2.5
        #10;
        $display("Test 1: 1.5 + 2.5");
        $display("Result: %h", out);

        #10

        // Test 3: 5.0 + 2.0 = 3.0
        inp1 = 32'h40A00000; // 5.0
        inp2 = 32'h40000000; // 2.0
        #10;
        $display("Test 3: 5.0 - 2.0");
        $display("Result: %h", out);

        
        

        // Test 5: -3.0 + (-4.0) = -7.0
        inp1 = 32'hC0400000; // -3.0
        inp2 = 32'hC0800000; // -4.0
        #10;
        $display("Test 5: -3.0 + (-4.0)");
        $display("Result: %h", out);
      
      	inp1 = 32'h3fc00000;//1.5
      	inp2 = 32'hc0200000;//-2.5
		#10
      $display("Test 7: 1.5 - 2.5");
        $display("Result: %h", out);
        $finish;
    end

endmodule