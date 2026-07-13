// ============================================================================
// PARAMETERIZED SPI MASTER WITH FSM & CLOCK DIVIDER
// Supports all 4 SPI Modes, Multi-Slaves, and Configurable Data Width
// ============================================================================
module spi_master #(
    parameter DATA_WIDTH   = 8,         // Number of bits per transfer
    parameter NUM_SLAVES   = 4,         // Number of slave select lines
    parameter CLK_DIV_WIDTH = 8         // Bit width of the clock divider register
)(
    input  wire                      clk,       // System clock
    input  wire                      rst_n,     // Active-low asynchronous reset
    
    // Control Interface
    input  wire                      start,     // Pulse high for 1 cycle to begin
    input  wire [NUM_SLAVES-1:0]     slave_sel, // One-hot slave selection
    input  wire [1:0]                spi_mode,  // SPI Mode: [1]=CPOL, [0]=CPHA
    input  wire [CLK_DIV_WIDTH-1:0]  clk_div,   // SCLK division factor: (System_Clk / (2 * clk_div))
    input  wire [DATA_WIDTH-1:0]     tx_data,   // Data to transmit
    
    // SPI Physical Interface
    output reg                       sclk,
    output reg                       mosi,
    output reg  [NUM_SLAVES-1:0]     cs_n,
    input  wire                      miso,
    
    // Status Interface
    output reg  [DATA_WIDTH-1:0]     rx_data,
    output reg                       done
);

    // ---- FSM States ----
    localparam STATE_IDLE      = 2'b00;
    localparam STATE_PREPARE   = 2'b01;
    localparam STATE_TRANSFER  = 2'b10;
    localparam STATE_DONE      = 2'b11;

    reg [1:0] current_state, next_state;

    // ---- Internal Registers ----
    reg [DATA_WIDTH-1:0]     shift_reg;
    reg [$clog2(DATA_WIDTH):0] bit_cnt; // Tracks number of shifted bits
    reg [CLK_DIV_WIDTH-1:0]  div_cnt;   // Clock divider counter
    reg                      sclk_en;   // Enables SCLK toggling
    reg                      sclk_edge; // Pulses high on every internal half-period tick
    reg                      sclk_prev; // Used to detect edges of sclk
    
    // Extract CPOL and CPHA configurations
    wire cpol = spi_mode[1];
    wire cpha = spi_mode[0];

    // ---- Clock Divider Logic ----
    // Generates internal execution ticks (`sclk_edge`) to toggle or sample data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt   <= 0;
            sclk_edge <= 1'b0;
        end else if (current_state == STATE_TRANSFER) begin
            if (div_cnt == clk_div - 1) begin
                div_cnt   <= 0;
                sclk_edge <= 1'b1;
            end else begin
                div_cnt   <= div_cnt + 1;
                sclk_edge <= 1'b0;
            end
        end else begin
            div_cnt   <= 0;
            sclk_edge <= 1'b0;
        end
    end

    // ---- FSM State Register ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= STATE_IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // ---- FSM Next State Logic ----
    always @(*) begin
        next_state = current_state;
        case (current_state)
            STATE_IDLE: begin
                if (start && (slave_sel != 0)) next_state = STATE_PREPARE;
            end
            STATE_PREPARE: begin
                // One cycle to pull CS_N low and setup base properties before clocking
                next_state = STATE_TRANSFER;
            end
            STATE_TRANSFER: begin
                // End transfer after shifting all bits (requires 2 ticks per bit)
                if (sclk_edge && (bit_cnt == DATA_WIDTH) && (sclk == (cpha ? cpol : ~cpol))) begin
                    next_state = STATE_DONE;
                end
            end
            STATE_DONE: begin
                next_state = STATE_IDLE;
            end
            default: next_state = STATE_IDLE;
        endcase
    end

    // ---- SPI Datapath and Control Output Logic ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk      <= 1'b0;
            mosi      <= 1'b0;
            cs_n      <= {NUM_SLAVES{1'b1}}; // All deselected
            rx_data   <= 0;
            done      <= 1'b0;
            shift_reg <= 0;
            bit_cnt   <= 0;
            sclk_prev <= 1'b0;
        end else begin
            done      <= 1'b0; // Default pulse constraint
            sclk_prev <= sclk;

            case (current_state)
                STATE_IDLE: begin
                    sclk <= cpol; // Match CPOL idle state dynamically
                    mosi <= 1'b0;
                    cs_n <= {NUM_SLAVES{1'b1}};
                    if (start && (slave_sel != 0)) begin
                        shift_reg <= tx_data;
                        bit_cnt   <= 0;
                    end
                end

                STATE_PREPARE: begin
                    cs_n <= ~slave_sel; // Drive target slave CS_N low
                    // CPHA=0 setup: Drive first bit out early before any clock edge
                    if (!cpha) begin
                        mosi <= shift_reg[DATA_WIDTH-1];
                    end
                end

                STATE_TRANSFER: begin
                    if (sclk_edge) begin
                        sclk <= ~sclk; // Toggle SPI Clock line
                        
                        // --- SPI Mode Sampling & Shifting Matrix ---
                        
                        // Condition A: Sample incoming MISO line
                        // Mode 0 & 2 (CPHA=0): Sample on leading/rising edge (sclk transitions away from cpol)
                        // Mode 1 & 3 (CPHA=1): Sample on trailing/falling edge (sclk transitions back to cpol)
                        if ((!cpha && (sclk == cpol)) || (cpha && (sclk != cpol))) begin
                            shift_reg <= {shift_reg[DATA_WIDTH-2:0], miso};
                            if (cpha) bit_cnt <= bit_cnt + 1;
                        end
                        
                        // Condition B: Drive outgoing MOSI line
                        // Mode 0 & 2 (CPHA=0): Update on trailing edge (sclk transitions back to cpol)
                        // Mode 1 & 3 (CPHA=1): Update on leading edge (sclk transitions away from cpol)
                        else if ((!cpha && (sclk != cpol)) || (cpha && (sclk == cpol))) begin
                            if (!cpha) begin
                                bit_cnt <= bit_cnt + 1;
                                if (bit_cnt < DATA_WIDTH - 1) begin
                                    mosi <= shift_reg[DATA_WIDTH-1];
                                end
                            end else begin
                                mosi <= shift_reg[DATA_WIDTH-1];
                            end
                        end
                    end
                end


            /*    STATE_TRANSFER: begin
    // End transfer right when the 8th bit's count/edge occurs (no extra pulse)
    if (sclk_edge && (bit_cnt == DATA_WIDTH-1) && (sclk == ~cpol)) begin
        next_state = STATE_DONE;
    end
end*/
                STATE_DONE: begin
                    cs_n    <= {NUM_SLAVES{1'b1}}; // Return CS_N high
                    done    <= 1'b1;
                    // For CPHA=0, final bit sample happens at the end of the final cycle
                    if (!cpha) begin
                        rx_data <= {shift_reg[DATA_WIDTH-2:0], miso};
                    end else begin
                        rx_data <= shift_reg;
                    end
                    sclk    <= cpol; // Ensure clock line retains CPOL state
                    mosi    <= 1'b0;
                end
               /* STATE_DONE: begin
    cs_n    <= {NUM_SLAVES{1'b1}};
    done    <= 1'b1;
    rx_data <= shift_reg;   // <-- was: (!cpha) ? {shift_reg[DATA_WIDTH-2:0], miso} : shift_reg
    sclk    <= cpol;
    mosi    <= 1'b0;
end*/
            endcase
        end
    end
endmodule
