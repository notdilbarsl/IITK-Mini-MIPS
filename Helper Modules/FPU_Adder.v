// Code your design here
`timescale 1ns/1ps


module floating_adder (
    input wire [31:0] inp1 , input [31:0] inp2 , output reg[31:0] out
);
    reg signa , signb;
    reg [7:0] exponenta, exponentb ;
    reg [7:0] diff;
    reg [24:0] ruffa , ruffb;
  reg[24:0] ans;
  reg[24:0] manta, mantb ;
    reg into;
    always @(inp1 or inp2) begin
      into = 0;
        signa = inp1[31]; signb = inp2[31];
        exponenta = inp1[31:23]; exponentb = inp2[31:23];
        manta = {1'b1, inp1[22:0]}; mantb = {1'b1, inp2[22:0]};
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