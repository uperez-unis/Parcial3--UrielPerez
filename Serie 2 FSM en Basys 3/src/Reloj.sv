`timescale 100ns / 1ps


module Reloj(
    input logic clk, reset,
    output logic [3:0] hora_d, hora_u, min_d, min_u
);

    logic [25:0] count = 0;
    logic [5:0] seg = 0;
    logic [3:0] uni_min = 0;
    logic [3:0] dec_min = 0;
    logic [3:0] uni_h = 0;
    logic [3:0] dec_h = 0;


    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            count <= 0;
            seg <= 0;
            uni_min   <= 0;
            dec_min   <= 0;
            uni_h  <= 0;
            dec_h  <= 0;
        end 
        else begin
            
            if (count == 1000000 - 1) begin
                count <= 0;
                seg <= seg + 1;
            end 
            else begin
                count <= count + 1;
            end

            
            if (seg == 60) begin
                seg <= 0;
                uni_min <= uni_min + 1;

                if (uni_min == 9) begin
                    uni_min <= 0;
                    dec_min <= dec_min + 1;

                    if (dec_min == 5) begin
                        dec_min  <= 0;
                        uni_h <= uni_h + 1;

                        if (uni_h == 9) begin
                            uni_h <= 0;
                            dec_h <= dec_h + 1;
                        end

                        if (uni_h == 3 && dec_h == 2) begin
                            uni_h <= 0;
                            dec_h <= 0;
                        end
                    end
                end
            end
        end
    end

    
    assign min_u  = uni_min;
    assign min_d  = dec_min;
    assign hora_u = uni_h;
    assign hora_d = dec_h;

endmodule
