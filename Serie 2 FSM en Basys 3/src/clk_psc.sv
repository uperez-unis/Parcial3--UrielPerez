`timescale 100ns / 1ps

module clk_psc(
    input logic clk,
    output logic clk_scaled
    );
    
    logic [31:0]myreg;
    always @(posedge clk)
        myreg +=1;
        
    assign clk_scaled = myreg[28];
endmodule
