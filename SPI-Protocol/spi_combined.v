module spi_combined (
    input wire         PCLK,
    input wire         PRESETn,
    input wire [1:0]   spi_mode,
    input wire         ss,
    input wire         spiswai,
    input wire [2:0]   spr,
    input wire [2:0]   sppr,
    input wire         cpha,
    input wire         cpol,
    output reg         sclk,
    output reg         flags_low,
    output reg         flags_high,
    output reg         flag_low,
    output reg         flag_high,
    output  [11:0]  BaudRateDivisor
);

    // ------------------ spi_clk_gen logic ------------------
    reg [11:0] count;
    reg        pre_sclk;
    assign BaudRateDivisor = (sppr+1)*(1<<(spr+1));

    wire mode_00 = (spi_mode == 2'b00);
    wire mode_01 = (spi_mode == 2'b01);
    wire mode_sel_clk = mode_00 | mode_01;

    wire en = mode_sel_clk & ~ss & ~spiswai;
    wire [11:0] BaudRateDiv_plus1 = BaudRateDivisor + 12'd1;
    wire count_match = (count == BaudRateDiv_plus1);

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            count     <= 12'd0;
            pre_sclk  <= 1'b0;
            sclk      <= 1'b0;
        end else begin
            if (en) begin
                if (count_match) begin
                    count     <= 12'd0;
                    pre_sclk  <= ~sclk;
                    sclk      <= ~sclk;
                end else begin
                    count <= count + 12'd1;
                end
            end else begin
                count <= 12'd0;
                sclk  <= pre_sclk;
            end
        end
    end

    // ------------------ spi_flag_gen logic ------------------
    wire mode_sel_flag = (~cpha & cpol) | (cpha & ~cpol);

    // FLAG LOW 10
    wire ux1_low_10_out = (count == (BaudRateDivisor - 12'd2)) ? 1'b1 : 1'b0;
    wire ux2_low_10_out = sclk ? 1'b0 : ux1_low_10_out;
    wire ux3_low_10_out = mode_sel_flag ? flags_low : ux2_low_10_out;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            flags_low <= 1'b0;
        else
            flags_low <= ux3_low_10_out;
    end

    // FLAG HIGH 10
    wire ux1_high_10_out = (count == (BaudRateDivisor - 12'd2)) ? 1'b1 : 1'b0;
    wire ux2_high_10_out = sclk ? ux1_high_10_out : 1'b0;
    wire ux3_high_10_out = mode_sel_flag ? ux2_high_10_out : flags_high;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            flags_high <= 1'b0;
        else
            flags_high <= ux3_high_10_out;
    end

    // FLAG LOW 1
    wire ux1_low_1_out = (count == (BaudRateDivisor - 12'd1)) ? 1'b1 : 1'b0;
    wire ux2_low_1_out = sclk ? 1'b0 : ux1_low_1_out;
    wire ux3_low_1_out = mode_sel_flag ? flag_low: ux2_low_1_out;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            flag_low <= 1'b0;
        else
            flag_low <= ux3_low_1_out;
    end

    // FLAG HIGH 1
    wire ux1_high_1_out = (count == (BaudRateDivisor - 12'd1)) ? 1'b1 : 1'b0;
    wire ux2_high_1_out = sclk ? ux1_high_1_out : 1'b0;
    wire ux3_high_1_out = !mode_sel_flag ? flag_high : ux2_high_1_out;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn)
            flag_high <= 1'b0;
        else
            flag_high <= ux3_high_1_out;
    end

endmodule

module spi_combined_tb;

    reg         PCLK;
    reg         PRESETn;
    reg  [1:0]  spi_mode;
    reg         ss;
    reg         spiswai;
    reg         cpha;
    reg         cpol;
    reg  [2:0]  spr;
    reg  [2:0]  sppr;

    wire        sclk;
    wire        flags_low;
    wire        flags_high;
    wire        flag_low;
    wire        flag_high;
    wire [11:0] BaudRateDivisor;

    // Instantiate DUT
    spi_combined uut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .spi_mode(spi_mode),
        .ss(ss),
        .spiswai(spiswai),
        .spr(spr),
        .sppr(sppr),
        .cpha(cpha),
        .cpol(cpol),
        .sclk(sclk),
        .flags_low(flags_low),
        .flags_high(flags_high),
        .flag_low(flag_low),
        .flag_high(flag_high),
        .BaudRateDivisor(BaudRateDivisor)
    );

    // Clock generation
    initial PCLK = 0;
    always #5 PCLK = ~PCLK;  // 100MHz

    // Tasks
    task reset_dut();
        begin
            PRESETn = 0;
            #20;
            PRESETn = 1;
        end
    endtask

    task set_spi_config(
        input [1:0] mode,
        input       swai,
        input       CPOL,
        input       CPHA,
        input [2:0] SPR_val,
        input [2:0] SPPR_val
    );
        begin
            spi_mode = mode;
            spiswai  = swai;
            cpol     = CPOL;
            cpha     = CPHA;
            spr      = SPR_val;
            sppr     = SPPR_val;
        end
    endtask

    task simulate_cycles(input integer num_cycles);
        begin
            repeat (num_cycles) @(posedge PCLK);
        end
    endtask

    initial begin
        // Init inputs
        spi_mode  = 2'b00;
        ss        = 1;  // inactive at first
        spiswai   = 0;
        spr       = 3'd0;
        sppr      = 3'd0;
        cpha      = 0;
        cpol      = 0;

        reset_dut();

        // -------- SPI Config --------
        // BaudRateDivisor = (sppr+1) * 2^(spr+1)
        // Example: sppr = 0, spr = 1 => (0+1) * 2^2 = 1 * 4 = 4
        //          sppr = 1, spr = 1 => (1+1) * 2^2 = 2 * 4 = 8
        //          sppr = 0, spr = 2 => (0+1) * 2^3 = 1 * 8 = 8
        // Let's pick sppr=0, spr=2 ? BaudRateDivisor = 8
        set_spi_config(2'b00, 0, 0, 0, 3'd2, 3'd0); // baud ? 8
        #5;
        ss = 0;  // activate slave
        simulate_cycles(100);  // observe clock and flags

        $finish;
    end

endmodule
