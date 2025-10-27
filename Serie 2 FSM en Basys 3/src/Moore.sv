`timescale 100ns / 1ps

module Moore(
 input  logic clk, reset, clk2,
 input logic select,    
 input  logic A, B,     
 output logic [1:0] C, 
 output logic [3:0] enabled,
 output logic [6:0] ag    
 
);

typedef enum logic [1:0] {S0,S1,S2,S3} statetype;
statetype state, nextstate;

typedef enum logic [1:0] {Q00, Q05, Q10, Q15} outtype;
outtype l;


logic [3:0] dread = 4'b0000;                
logic [3:0] hora_d, hora_u, min_d, min_u;   
logic [3:0] d0, d1, d2, d3;                 

always_ff @(posedge clk2 or posedge reset)
    if (reset) begin state <= S0; dread <= 0; end
    else 
        begin
            state <= nextstate;
            
            if(B && ~A) 
                begin
                    if(dread < 3)dread <= dread + 1;
                    else dread <= dread;
                end
            else if(~B && A) dread  <= 0;
            else dread  <= dread;
    end
    
always_comb
    case (state)    
        S0: if (B && ~A) nextstate = S1;
            else nextstate = S0;

        S1: if (~B && A) nextstate = S0;
            else if (B && ~A) nextstate = S2;
            else nextstate = S1;

        S2: if (~B && A) nextstate = S0;
            else if (B && ~A) nextstate = S3;
            else nextstate = S2;

        S3: if (~B && A) nextstate = S0;
            else nextstate = S3;

        default: nextstate = S0;
    endcase
    
always_comb 
    case (state)
        S0: l = Q00;
        S1: l = Q05;
        S2: l = Q10;
        S3: l = Q15;
    endcase
    
assign C = state;

Reloj clock(
    .clk(clk),
    .reset(reset),
    .hora_d(hora_d),
    .hora_u(hora_u),
    .min_d(min_d),
    .min_u(min_u)
    );
    
always_comb 
begin
    if(select == 0)
        begin
            d0 = dread;
            d1 = 4'b0000;
            d2 = 4'b0000;
            d3 = 4'b0000;
        end
    else
        begin
            d0 = min_u;
            d1 = min_d;
            d2 = hora_u;
            d3 = hora_d;   
        end
end

Display7seg Display(
    .clk(clk),
    .d0(d0),
    .d1(d1),
    .d2(d2),
    .d3(d3),
    .enabled(enabled),
    .ag(ag)
);


endmodule

