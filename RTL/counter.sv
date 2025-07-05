module counter #(
    parameter int MAX_COUNT = 255  // Maximum count value
)(
    input  logic clk,
    input  logic rst_n,              // synchronous reset
    input  logic en,               // enable signal
    output logic [$clog2(MAX_COUNT+1)-1:0] count
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        count <= '0;
    else if (en) begin
        if (count == MAX_COUNT)
            count <= '0;
        else
            count <= count + 1;
    end
end

endmodule
