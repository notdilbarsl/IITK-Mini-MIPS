`timescale 1ns/1ps

module Floating_point_to_Binary (
    input [31:0] floating_input , output reg [31:0] bin_output
);
    reg sign;
    reg [32:0] mantissa;
    reg [7 : 0] exponent;
    reg [7:0] diff;
    integer i;
    always @(floating_input) begin
        {sign, exponent, mantissa[31:9]} = floating_input;
        mantissa[32] = 1;
        mantissa[8:0] = 0;
        bin_output = 0;
        diff = exponent - 127;
        if (diff > 30) begin
            bin_output = 0;
        end
        else if(diff < 0) begin
            bin_output = 0;
        end
        else begin
            for(i = diff + 1 ; i > -1 ; i = i - 1) begin
                bin_output[i] = mantissa[32 + (i - (diff  + 1))];
            end
        end
    end

    
endmodule


module Binary_to_FloatingPoint (
    input [31:0] bin_input , output reg[31:0] floating_output
);
    reg[22:0] mantissa;
    reg sign;
    reg [7:0] exponent;
  reg[4:0] i;
    genvar j;
    always @(bin_input) begin
        sign = 0;
        exponent = 31;
        if (bin_input == 0) begin
            floating_output = 32'h00000000;
        end
        else begin
          for (i = 31; bin_input[i] == 0; i = i - 1) begin
            exponent = i;
          end;

            exponent = exponent - 1;

            mantissa = 0;
            if (i >= 23) begin
                for (integer j = 0; j < 23; j = j + 1) begin
                    mantissa[22 - j] = bin_input[i - 1 - j];
                end
            end 
            else begin
                for(integer j = exponent - 1 ; j >= 0 ; j = j - 1) begin
                    mantissa[22 + (j - (exponent - 1))] = bin_input[j];
                end
            end
            exponent = exponent + 127;
            floating_output = {sign , exponent, mantissa};
        
        end
    end
endmodule