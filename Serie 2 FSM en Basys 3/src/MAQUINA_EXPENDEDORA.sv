`timescale 1ns / 1ps

module MAQUINA_EXPENDEDORA(
    input logic clk, reset,
    input logic [3:0]sw,
    input  logic btnU,
    output logic [1:0] P,
    output logic Confimado,
    output logic [3:0] an,
    output logic [6:0] seg,
    output logic dp
    );
    
logic clk2;
logic select = 1'b0;
logic btnU_d;
logic [1:0] Credit;

always_ff @(posedge clk or posedge reset) begin
    if (reset)
        select <= 1'b0;
    else begin
        
        if (btnU && !btnU_d)
            select <= ~select;
        btnU_d <= btnU;  
    end
end

    

clk_psc clk_scl (
    .clk(clk),
    .clk_scaled(clk2)
);
   
Moore FSM_moore (
    .clk(clk),
    .clk2(clk2),
    .reset(reset),
    .select(select),
    .A(sw[1]),
    .B(sw[0]),
    .C(Credit),
    .enabled(an),
    .ag(seg)
   );
   
FSM_Mealy Mealy (
    .clk(clk2),
    .reset(reset),
    .B(Credit),
    .C(sw[3:2]),
    .P(P[1:0]),
    .A(Confimado)
   );
   
   //assign dp = 1'b1;
endmodule
