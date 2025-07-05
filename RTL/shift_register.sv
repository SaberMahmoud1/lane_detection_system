module shift_register #(
    parameter int DEPTH = 6  // Number of bits to shift
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,       // Enable signal added
    input  logic din,
    output logic [DEPTH-1:0] dout
);

    logic [DEPTH-1:0] shift_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= '0;
        end else if (enable) begin
            shift_reg <= {shift_reg[DEPTH-2:0], din};
        end
    end

    assign dout = shift_reg;

endmodule
