// ============================================================
// TESTBENCH V2
// Tests: All 4 SPI modes × 3 slaves, DATA_W=8
// MISO mux: only selected slave drives MISO (others tri-state)
// ============================================================
`timescale 1ns/1ps

module tb_spi;

    parameter DATA_W     = 8;
    parameter NUM_SLAVES = 3;
    parameter CLK_DIV    = 4;

    reg clk, rst_n, start;
    initial clk = 0;
    always #5 clk = ~clk;

    reg [1:0]                        mode;
    reg [$clog2(NUM_SLAVES)-1:0]     slave_sel;
    reg [DATA_W-1:0]                 master_tx;
    reg [DATA_W-1:0]                 slave_tx_d [0:NUM_SLAVES-1];

    wire                             sclk, mosi;
    wire [NUM_SLAVES-1:0]            cs_n;
    wire [DATA_W-1:0]                master_rx;
    wire                             master_done;

    // Each slave has its own MISO wire; mux selects correct one
    wire [NUM_SLAVES-1:0]            miso_w;
    wire [DATA_W-1:0]                slave_rx [0:NUM_SLAVES-1];
    wire [NUM_SLAVES-1:0]            slave_done_w;

    // MISO mux: pick the active slave's line (cs_n[i]==0 means that slave is selected)
    wire miso = cs_n[0] ? (cs_n[1] ? miso_w[2] : miso_w[1]) : miso_w[0];

    // ---- Master ----
    spi_master #(.DATA_W(DATA_W),.NUM_SLAVES(NUM_SLAVES),.CLK_DIV(CLK_DIV)) u_master (
        .clk(clk), .rst_n(rst_n), .start(start), .mode(mode),
        .slave_sel(slave_sel), .tx_data(master_tx), .miso(miso),
        .rx_data(master_rx), .done(master_done),
        .sclk(sclk), .mosi(mosi), .cs_n(cs_n)
    );

    // ---- 3 Slaves ----
    genvar i;
    generate
        for (i = 0; i < NUM_SLAVES; i = i+1) begin : slv
            spi_slave #(.DATA_W(DATA_W)) u (
                .sclk(sclk), .cs_n(cs_n[i]), .mode(mode),
                .mosi(mosi), .tx_data(slave_tx_d[i]),
                .miso(miso_w[i]), .rx_data(slave_rx[i]), .done(slave_done_w[i])
            );
        end
    endgenerate

    // ---- Task: single test ----
    task run_test;
        input [1:0]        t_mode;
        input [1:0]        t_slave;
        input [DATA_W-1:0] t_mtx;
        input [DATA_W-1:0] t_stx;
        begin
            // Reset between tests for clean state
            rst_n = 0; #20; rst_n = 1; #20;

            slave_tx_d[0] = 8'h00;
            slave_tx_d[1] = 8'h00;
            slave_tx_d[2] = 8'h00;
            slave_tx_d[t_slave] = t_stx;

            mode      = t_mode;
            slave_sel = t_slave;
            master_tx = t_mtx;

            @(posedge clk); #1;
            start = 1;
            @(posedge clk); #1;
            start = 0;

            wait(master_done);
            @(posedge clk); #1;

            $display("Mode=%0d Slv=%0d | M->S: sent=%02X got=%02X %s | S->M: sent=%02X got=%02X %s",
                t_mode, t_slave,
                t_mtx,  slave_rx[t_slave], (slave_rx[t_slave]==t_mtx) ? "PASS":"FAIL",
                t_stx,  master_rx,          (master_rx==t_stx)         ? "PASS":"FAIL");

            #50;
        end
    endtask

    integer j;
    initial begin
        $dumpfile("spi_v2_wave.vcd");
        $dumpvars(0, tb_spi_v2);

        for (j=0; j<NUM_SLAVES; j=j+1) slave_tx_d[j] = 0;
        rst_n = 0; start = 0; mode = 0; slave_sel = 0; master_tx = 0;
        #30; rst_n = 1; #20;

        $display("============ SPI V2 Full Test Suite ============");

        // -- Mode 0 (CPOL=0, CPHA=0) --
        run_test(2'd0, 2'd0, 8'hA5, 8'h3C);
        run_test(2'd0, 2'd1, 8'hDE, 8'hAD);
        run_test(2'd0, 2'd2, 8'hBE, 8'hEF);
        // -- Mode 1 (CPOL=0, CPHA=1) --
        run_test(2'd1, 2'd0, 8'h12, 8'h34);
        run_test(2'd1, 2'd1, 8'hAB, 8'hCD);
        // -- Mode 2 (CPOL=1, CPHA=0) --
        run_test(2'd2, 2'd0, 8'h56, 8'h78);
        run_test(2'd2, 2'd2, 8'hF0, 8'h0F);
        // -- Mode 3 (CPOL=1, CPHA=1) --
        run_test(2'd3, 2'd0, 8'h9A, 8'hBC);
        run_test(2'd3, 2'd1, 8'h55, 8'hAA);

        $display("================================================");
        #100; $finish;
    end

endmodule
