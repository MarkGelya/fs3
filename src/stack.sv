module lifo8x8(
    input logic clk,
    input logic rst,
    input logic push,
    input logic pop,
    output logic [3:0] led,
    input logic [7:0] in,
    output logic valid,
    output logic full,
    output logic [7:0] out
    );

    reg [7:0] reg_array [0:7];
    reg [3:0] counter;
    
    assign led = counter;
    assign out = reg_array[counter-1];
    assign valid = |counter;
    assign full = counter[3]; 

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            counter <= 'b0;
            for (int i = 0; i <= 7; i++)
                reg_array[i] <= 'b0;
        end else begin
            if (push) begin
                if (pop)
                    reg_array[counter-1] <= in;
                else
                    reg_array[counter] <= in;
            end
            if (push ^ pop)
                counter <= counter + (push ? 1 : -1);
        end
    end

endmodule
