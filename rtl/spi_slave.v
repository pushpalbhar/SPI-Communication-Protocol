// ============================================================
// SPI SLAVE- All 4 Modes, Parameterized
//
// Mode | SAMPLE edge | SHIFT edge
//  0   | posedge     | negedge
//  1   | negedge     | posedge
//  2   | negedge     | posedge   (CPOL=1 inverts clock polarity)
//  3   | posedge     | negedge
//
// sample_on_pos = !(cpol ^ cpha)
// shift_on_pos  =  (cpol ^ cpha)
//
// CPHA=0: MSB pre-loaded at CS assertion (before any clock edge).
// CPHA=1: On the FIRST shift edge, drive MSB onto MISO without
//         consuming the register. Subsequent shifts consume normally.
// ============================================================

module spi_slave #(
    parameter DATA_W = 8
)(
    input  wire              sclk,
    input  wire              cs_n,
    input  wire [1:0]        mode,
    input  wire              mosi,
    input  wire [DATA_W-1:0] tx_data,
    output reg               miso,
    output reg  [DATA_W-1:0] rx_data,
    output reg               done
);

    wire cpol = mode[1];
    wire cpha = mode[0];

    reg [DATA_W-1:0]         rx_sr;
    reg [DATA_W-1:0]         tx_sr;
    reg [$clog2(DATA_W+1):0] bit_cnt;
    reg                      active;
    reg                      first_shift;

    // ---- CS assert: initialise ----
    always @(negedge cs_n) begin
        tx_sr       <= tx_data;
        rx_sr       <= 0;
        bit_cnt     <= 0;
        done        <= 0;
        active      <= 1;
        first_shift <= cpha;
        miso        <= (!cpha) ? tx_data[DATA_W-1] : 1'b0;
    end

    always @(posedge cs_n) active <= 0;

    // ---- SAMPLE task (inline in each always) ----
    // ---- SHIFT  task (inline in each always) ----

    // ---- posedge SCLK ----
    always @(posedge sclk) begin
        if (!cs_n && active) begin
            if (!(cpol ^ cpha)) begin
                // SAMPLE (Mode 0 or 3)
                rx_sr   <= {rx_sr[DATA_W-2:0], mosi};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == DATA_W - 1) begin
                    rx_data <= {rx_sr[DATA_W-2:0], mosi};
                    done    <= 1;
                    active  <= 0;
                end
            end else begin
                // SHIFT (Mode 1 or 2)
                if (first_shift) begin
                    miso        <= tx_sr[DATA_W-1]; // present MSB, don't consume
                    first_shift <= 0;
                end else begin
                    tx_sr <= {tx_sr[DATA_W-2:0], 1'b0};
                    miso  <= tx_sr[DATA_W-2];
                end
            end
        end
    end

    // ---- negedge SCLK ----
    always @(negedge sclk) begin
        if (!cs_n && active) begin
            if (cpol ^ cpha) begin
                // SAMPLE (Mode 1 or 2)
                rx_sr   <= {rx_sr[DATA_W-2:0], mosi};
                bit_cnt <= bit_cnt + 1;
                if (bit_cnt == DATA_W - 1) begin
                    rx_data <= {rx_sr[DATA_W-2:0], mosi};
                    done    <= 1;
                    active  <= 0;
                end
            end else begin
                // SHIFT (Mode 0 or 3)
                // For Mode 3 (cpol=1,cpha=1): shift is on negedge.
                // first_shift applies here too (cpha=1).
                if (first_shift) begin
                    miso        <= tx_sr[DATA_W-1]; // present MSB, don't consume
                    first_shift <= 0;
                end else begin
                    tx_sr <= {tx_sr[DATA_W-2:0], 1'b0};
                    miso  <= tx_sr[DATA_W-2];
                end
            end
        end
    end
endmodule
