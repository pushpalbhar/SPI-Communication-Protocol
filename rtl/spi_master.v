// ============================================================
// SPI MASTER - All 4 Modes, Parameterized, Multi-Slave, CLK_DIv
// KEY DESIGN DECISION (fixes the sampling bug):
//   SCLK is a *registered output* driven from sys-clk.
//   We use an internal flag `sclk_int` that toggles on tick.
//   Master samples MISO and updates MOSI based on sclk_int
//   transitions BEFORE they appear on the output port - i.e.,
//   we look at what sclk_int IS NOW to decide what to do NOW,
//   and the port `sclk` gets the new value this same cycle.
//   This avoids the 1-cycle mismatch.
//
// FSM:
//   IDLE → CS_LOW → TRANSFER → CS_HIGH → DONE → IDLE
// TRANSFER details:
//   Each bit = 2 ticks (half0 + half1).
//   half0: first  edge - action = sample  if CPHA=0, shift if CPHA=1
//   half1: second edge - action = shift   if CPHA=0, sample if CPHA=1
//   After DATA_W samples, leave TRANSFER.
// ============================================================

module spi_master #(
    parameter DATA_W     = 8,
    parameter NUM_SLAVES = 3,
    parameter CLK_DIV    = 4       // SCLK freq = sys_clk / (2*CLK_DIV)
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          start,
    input  wire [1:0]                    mode,       // {CPOL, CPHA}
    input  wire [$clog2(NUM_SLAVES)-1:0] slave_sel,
    input  wire [DATA_W-1:0]             tx_data,
    input  wire                          miso,

    output reg  [DATA_W-1:0]             rx_data,
    output reg                           done,
    output reg                           sclk,
    output reg                           mosi,
    output reg  [NUM_SLAVES-1:0]         cs_n
);

    // ---- Latched config ----
    reg                          cpol, cpha;
    reg [$clog2(NUM_SLAVES)-1:0] slv;

    // ---- FSM ----
    localparam S_IDLE     = 3'd0;
    localparam S_CS_LOW   = 3'd1;
    localparam S_TRANSFER = 3'd2;
    localparam S_CS_HIGH  = 3'd3;
    localparam S_DONE     = 3'd4;
    reg [2:0] state;

    // ---- Datapath ----
    reg [DATA_W-1:0]        tx_sr;      // TX shift register (MSB first)
    reg [DATA_W-1:0]        rx_sr;      // RX shift register
    reg [$clog2(DATA_W):0]  bit_cnt;    // how many bits sampled so far
    reg                     half;       // 0=first half of bit, 1=second half

    // ---- Clock divider: generate 1-cycle tick every CLK_DIV cycles ----
    reg [$clog2(CLK_DIV):0] div_cnt;
    reg                      tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
            tick    <= 0;
        end else begin
            tick <= 0;
            if (div_cnt == CLK_DIV - 1) begin
                div_cnt <= 0;
                tick    <= 1;
            end else
                div_cnt <= div_cnt + 1;
        end
    end

    // ============================================================
    // FSM + Datapath
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cs_n    <= {NUM_SLAVES{1'b1}};
            sclk    <= 0;
            mosi    <= 0;
            done    <= 0;
            tx_sr   <= 0;
            rx_sr   <= 0;
            rx_data <= 0;
            bit_cnt <= 0;
            half    <= 0;
            cpol    <= 0; cpha <= 0; slv <= 0;
        end else begin
            done <= 0;

            case (state)

                // ------------------------------------------------
                S_IDLE: begin
                    cs_n <= {NUM_SLAVES{1'b1}};
                    sclk <= mode[1];       // idle level = CPOL
                    if (start) begin
                        cpol    <= mode[1];
                        cpha    <= mode[0];
                        slv     <= slave_sel;
                        tx_sr   <= tx_data;
                        rx_sr   <= 0;
                        bit_cnt <= 0;
                        half    <= 0;
                        state   <= S_CS_LOW;
                    end
                end

                // ------------------------------------------------
                // Assert CS, present first bit if CPHA=0
                S_CS_LOW: begin
                    if (tick) begin
                        cs_n       <= {NUM_SLAVES{1'b1}};
                        cs_n[slv]  <= 1'b0;
                        sclk       <= cpol;          // keep idle level
                        // CPHA=0: master presents MSB NOW (before first edge)
                        // CPHA=1: master presents MSB on the first clock edge
                        mosi  <= cpha ? 1'b0 : tx_sr[DATA_W-1];
                        state <= S_TRANSFER;
                    end
                end

                // ------------------------------------------------
                // Shift DATA_W bits
                S_TRANSFER: begin
                    if (tick) begin
                        sclk <= ~sclk;   // toggle SCLK
                        half <= ~half;

                        if (half == 0) begin
                            // ---- First edge of this bit ----
                            if (!cpha) begin
                                // Mode 0/2: SAMPLE on first edge
                                rx_sr   <= {rx_sr[DATA_W-2:0], miso};
                                bit_cnt <= bit_cnt + 1;
                            end else begin
                                // Mode 1/3: SHIFT (present next bit) on first edge
                                // For first bit: present tx_sr[DATA_W-1]
                                // For subsequent: tx_sr was already shifted, present MSB
                                mosi  <= tx_sr[DATA_W-1];
                                tx_sr <= {tx_sr[DATA_W-2:0], 1'b0};
                            end
                        end else begin
                            // ---- Second edge of this bit ----
                            if (!cpha) begin
                                // Mode 0/2: SHIFT (present next bit) on second edge
                                mosi  <= tx_sr[DATA_W-2];    // next bit
                                tx_sr <= {tx_sr[DATA_W-2:0], 1'b0};
                            end else begin
                                // Mode 1/3: SAMPLE on second edge
                                rx_sr   <= {rx_sr[DATA_W-2:0], miso};
                                bit_cnt <= bit_cnt + 1;
                            end

                            // After DATA_W samples, done
                            if (bit_cnt == (cpha ? DATA_W-1 : DATA_W)) begin
                                state <= S_CS_HIGH;
                            end
                        end
                    end
                end

                // ------------------------------------------------
                S_CS_HIGH: begin
                    if (tick) begin
                        cs_n  <= {NUM_SLAVES{1'b1}};
                        sclk  <= cpol;
                        mosi  <= 0;
                        state <= S_DONE;
                    end
                end

                // ------------------------------------------------
                S_DONE: begin
                    rx_data <= rx_sr;
                    done    <= 1;
                    state   <= S_IDLE;
                end

            endcase
        end
    end

endmodule
