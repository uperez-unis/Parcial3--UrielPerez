`timescale 100ns / 1ps

module FSM_Mealy(


    input logic clk, reset, [1:0] B, [1:0] C,
    output logic [1:0] P, 
    output logic A
    );
    
typedef enum logic [1:0] {S0, S1, S2, S3} statetype;
statetype current_state, next_state;

typedef enum logic [1:0] {nada, Pepsi, Coca, RedBull} outtype;
outtype l;

always_ff @(posedge clk, posedge reset)
    if (reset) current_state <= S0;
    else current_state <= next_state;
    

always_comb begin
    next_state = current_state;
    case (current_state)
        S0: if ((B == 2'b01 && C == 2'b00) || (B == 2'b01 && C == 2'b10)|| (B == 2'b01 && C == 2'b11) || (B == 2'b10 && C == 2'b01) || (B == 2'b11 && C == 2'b10)) 
            next_state = S1;
            
            else if ((B == 2'b10 && C == 2'b00) || (B == 2'b10 && C == 2'b11) || (B == 2'b11 && C == 2'b01))
            next_state = S2;
            
            else if (B == 2'b11 && C == 2'b00)
            next_state = S3;
            
            else next_state = S0;
             
        S1: if ((B == 2'b00 && C == 2'b01) || (B == 2'b01 && C == 2'b10) || (B == 2'b10 && C == 2'b11)) 
            next_state = S0;    
            
            else if((B == 2'b01 && C == 2'b00) || (B == 2'b01 && C == 2'b11) || (B == 2'b10 && C == 2'b01))
            next_state = S2;
            
            else if(B == 2'b10 && C == 2'b00)
            next_state = S3;
            
            else next_state = S1;
            
         S2: if ((B == 2'b00 && C == 2'b10) || (B == 2'b01 && C == 2'b11)) 
            next_state = S0;
            
            else if ((B == 2'b00 && C == 2'b01) || (B == 2'b01 && C == 2'b10)) 
            next_state = S1;
            
            else if (B == 2'b01 && C == 2'b00)
            next_state = S3;
            
            else next_state = S2;
            
         S3: if (B == 2'b00 && C == 2'b11) 
            next_state = S0;
            
            else if (B == 2'b00 && C == 2'b10) 
            next_state = S1;
            
            else if (B == 2'b00 && C == 2'b01)
            next_state = S2;
            
            else next_state = S3;
            
              
        default: next_state = S0;
    endcase
end


// Output Logic 
always_comb begin
    P = nada;
    A = 1'b0;

    case (current_state)
        S0: begin
            if ((B == 2'b00 && C == 2'b00) || (B == 2'b01 && C == 2'b00) || (B == 2'b10 && C == 2'b00) || (B == 2'b11 && C == 2'b00)) begin
                P = 2'b00;  A = 1'b0;
            end
            else if (B == 2'b00 && C == 2'b01) begin
                P = 2'b01;  A = 1'b0;
            end
            else if ((B == 2'b00 && C == 2'b10) || (B == 2'b01 && C == 2'b10)) begin
                P = 2'b10;  A = 1'b0;
            end
            else if ((B == 2'b00 && C == 2'b11) || (B == 2'b01 && C == 2'b11) || (B == 2'b10 && C == 2'b11)) begin
                P = 2'b11;  A = 1'b0;
            end
            else if ((B == 2'b01 && C == 2'b01) || (B == 2'b10 && C == 2'b01) || (B == 2'b11 && C == 2'b01)) begin
                P = 2'b01;  A = 1'b1;
            end
            else if ((B == 2'b10 && C == 2'b10) || (B == 2'b11 && C == 2'b10)) begin
                P = 2'b10;  A = 1'b1;
            end
            else if ((B == 2'b11 && C == 2'b11) || (B == 2'b11 && C == 2'b10)) begin
                P = 2'b11;  A = 1'b1;
            end
        end

        S1: begin
            if (B == 2'b00 && C == 2'b10) begin
                P = 2'b10;  A = 1'b0;
            end
            else if ((B == 2'b00 && C == 2'b11) || (B == 2'b01 && C == 2'b11)) begin
                P = 2'b11;  A = 1'b0;
            end
            else if ((B == 2'b01 && C == 2'b01) || (B == 2'b00 && C == 2'b01)) begin
                P = 2'b01;  A = 1'b1;
            end
            else if ((B == 2'b10 && C == 2'b10) || (B == 2'b01 && C == 2'b10)) begin
                P = 2'b10;  A = 1'b1;
            end
            else if (B == 2'b10 && C == 2'b11) begin
                P = 2'b11;  A = 1'b1;
            end
            else begin 
            P = 2'b00;  A = 1'b0; 
            end
        end
        
        S2: begin
            if (B == 2'b00 && C == 2'b11) begin
                P = 2'b11;  A = 1'b0;
            end
            else if ((B == 2'b01 && C == 2'b01) || (B == 2'b00 && C == 2'b01)) begin
                P = 2'b01;  A = 1'b1;
            end
            else if ((B == 2'b01 && C == 2'b10) || (B == 2'b00 && C == 2'b10)) begin
                P = 2'b10;  A = 1'b1;
            end
            else if (B == 2'b01 && C == 2'b11) begin
                P = 2'b11;  A = 1'b1;
            end
            else begin 
            P = 2'b00;  A = 1'b0; 
            end
        end
        
        S3: begin
            if (B == 2'b00 && C == 2'b01) begin
                P = 2'b01;  A = 1'b1;
            end
            else if (B == 2'b00 && C == 2'b10) begin
                P = 2'b10;  A = 1'b1;
            end
            else if (B == 2'b00 && C == 2'b11) begin
                P = 2'b11;  A = 1'b1;
            end
            else begin 
            P = 2'b00;  A = 1'b0; 
            end
        end
        
        
        default: begin
            P = 2'b00;
            A = 1'b0;
        end
    endcase
end


endmodule
