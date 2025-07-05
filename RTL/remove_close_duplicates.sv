module remove_close_duplicates #(
    parameter int IMG_WIDTH = 416,
    parameter int MAX_POINTS = 5,
    parameter int DIST_THRESHOLD = 50
)(
    input  logic                             clk,
    input  logic                             rst_n,
    input  logic                             start,
    input  logic [$clog2(IMG_WIDTH):0]       in_array     [MAX_POINTS],
    input  logic [$clog2(IMG_WIDTH):0]       importance   [MAX_POINTS],
    output logic [$clog2(IMG_WIDTH):0]       out_array    [MAX_POINTS]
);

    logic valid_mask [MAX_POINTS];
    logic [$clog2(IMG_WIDTH):0] abs_diff;

    logic [$clog2(IMG_WIDTH):0]       in_array_reg     [MAX_POINTS];
    logic [$clog2(IMG_WIDTH):0]       importance_reg   [MAX_POINTS];

    logic [$clog2(MAX_POINTS)-1:0]       first_index_counter,second_index_counter,stage_counter;

    logic inputs_sampled,first_index_counter_en,second_index_counter_en,stage_counter_en;

    typedef enum logic [1:0] {
        IDLE,
        SAMPLE,
        PROCESS,
        WRITE
    } state_t;

    state_t cs, ns;


    assign second_index_counter_en = inputs_sampled;
    assign first_index_counter_en = (second_index_counter >= MAX_POINTS - 1) ? 1 : 0;
    assign stage_counter_en = (cs == SAMPLE || cs == WRITE);
    

    // Instantiate the counter
    counter #(
        .MAX_COUNT(MAX_POINTS-1)
    ) f_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (first_index_counter_en),
        .count  (first_index_counter)
    );

    // Instantiate the counter 
    counter #(
        .MAX_COUNT(MAX_POINTS-1)
    ) s_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (second_index_counter_en),
        .count  (second_index_counter)
    );

        // Instantiate the stage counter 
    counter #(
        .MAX_COUNT(MAX_POINTS-1)
    ) st_counter (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (stage_counter_en),
        .count  (stage_counter)
    );


      // FSM transition logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)begin
            cs <= IDLE;
        end else begin
            cs <= ns;
        end
    end

    // FSM next cs logic
    always_comb begin
        case (cs)
            IDLE:begin
                if (start)begin
                    ns = SAMPLE;
                end else begin
                    ns = IDLE;
                end
            end
            SAMPLE: begin
                if (stage_counter == MAX_POINTS - 1) begin
                    ns = PROCESS;
                end else begin
                    ns = SAMPLE;
                end
            end
            PROCESS:begin
                if (first_index_counter == MAX_POINTS - 1 && second_index_counter == MAX_POINTS - 1)begin
                    ns = WRITE;
                end else begin
                    ns = PROCESS;
                end
            end
            WRITE:begin
                  if (stage_counter == MAX_POINTS - 1)begin
                    ns = IDLE;
                  end else begin
                    ns = WRITE;
                  end
            end
            default:begin
                ns = IDLE;
            end 
        endcase
    end

    //FSM operation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int k = 0; k < MAX_POINTS; k++) begin
                in_array_reg   [k] <= 0;
                importance_reg [k] <= 0;
                valid_mask     [k] <= 1'b1;
                out_array      [k] <= 0;
            end
        end else if (cs == SAMPLE) begin
            in_array_reg[stage_counter]   <= in_array[stage_counter];
            importance_reg[stage_counter] <= importance[stage_counter];
            valid_mask[stage_counter]     <= 1'b1;
        end else if (cs == PROCESS)begin
            // Compare each unique pair only once
                if (valid_mask[first_index_counter] && valid_mask[second_index_counter] && first_index_counter != second_index_counter) begin
                    if (abs_diff <= DIST_THRESHOLD) begin
                        if (importance_reg[first_index_counter] >= importance_reg[second_index_counter]) begin
                            valid_mask[second_index_counter] <= 1'b0;
                        end else begin
                            valid_mask[first_index_counter] <= 1'b0;
                        end
                    end
                end
        end else if (cs == WRITE) begin
             out_array[stage_counter] <= (valid_mask[stage_counter]) ? in_array_reg[stage_counter] : 0;
        end
    end

    assign  abs_diff = (in_array_reg[first_index_counter] > in_array_reg[second_index_counter]) ? 
                       (in_array_reg[first_index_counter] - in_array_reg[second_index_counter]) : 
                       (in_array_reg[second_index_counter] - in_array_reg[first_index_counter]);
endmodule
