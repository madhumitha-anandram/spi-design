module spi_final (
    input  PCLK,
    input  PRESETn,
    input  PWRITE,
    input  PSEL,
    input  PENABLE,
    input  miso,
    input  [2:0] PADDR,
    input  [7:0] PWDATA,
    output [7:0] PRDATA,
    output       PREADY,
    output       PSLVERR,
    output       sclk,
    output       ss,
    output       mosi,
    output       spi_interrupt_request
);

    wire tip;
    wire [1:0] spi_mode;
    wire spiswai, cpol, cpha, flag_high, flag_low, flags_high, flags_low;
    wire send_data, lsbfe, receive_data, mstr;
    wire [11:0] baud_rate_div;
    wire [7:0] data_mosi, data_miso;
    wire [2:0] spr, sppr;

    spi_combined BAUD_GEN (
        PCLK, PRESETn, spi_mode, ss, spiswai, spr, sppr, cpha, cpol, sclk,
        flag_high, flag_low, flags_high, flags_low, baud_rate_div
    );

    spi_slave_interface SLAVE_INTERFACE (
        PCLK, PRESETn, PADDR, PWRITE, PSEL, PENABLE, PWDATA, ss, data_miso,
        receive_data, tip, PRDATA, mstr, cpol, cpha, lsbfe, spiswai,
        spi_interrupt_request, PREADY, PSLVERR, send_data, data_mosi,
        spi_mode, spr, sppr
    );

    spi_control_logic SLAVE_SELECT (
        PCLK, PRESETn, send_data, spi_mode, spiswai, mstr, baud_rate_div,
        receive_data, tip, ss
    );

    spi_core_teporary SHIFT_REG (
        PCLK, PRESETn, receive_data, send_data, data_mosi, ss, cpha, cpol,
        miso, lsbfe, flags_low, flags_high, flag_low, flag_high, data_miso, mosi
    );

endmodule


// -------------------- Testbench --------------------

module spi_final_tb;

  reg         PCLK;
  reg         PRESETn;
  reg         miso;

  wire [7:0]  PRDATA;
  wire        PREADY, PSLVERR;
  wire        sclk, ss, mosi;
  wire        spi_interrupt_request;

  reg         PWRITE;
  reg         PSEL;
  reg         PENABLE;
  reg  [2:0]  PADDR;
  reg  [7:0]  PWDATA;

  spi_final DUT (
    .PCLK(PCLK),
    .PRESETn(PRESETn),
    .PWRITE(PWRITE),
    .PSEL(PSEL),
    .PENABLE(PENABLE),
    .miso(miso),
    .PADDR(PADDR),
    .PWDATA(PWDATA),
    .PRDATA(PRDATA),
    .PREADY(PREADY),
    .PSLVERR(PSLVERR),
    .sclk(sclk),
    .ss(ss),
    .mosi(mosi),
    .spi_interrupt_request(spi_interrupt_request)
  );

  // Clock generation
  initial PCLK = 0;
  always #5 PCLK = ~PCLK;

  // APB write task
  task apb_write(input [2:0] addr, input [7:0] data);
    begin
      @(negedge PCLK);
      PADDR   = addr;
      PWDATA  = data;
      PWRITE  = 1;
      PSEL    = 1;
      PENABLE = 0;
      @(negedge PCLK);
      PENABLE = 1;
      @(negedge PCLK);
      PSEL    = 0;
      PENABLE = 0;
    end
  endtask

  // APB read task
  task apb_read(input [2:0] addr);
    begin
      @(negedge PCLK);
      PADDR   = addr;
      PWRITE  = 0;
      PSEL    = 1;
      PENABLE = 0;
      @(negedge PCLK);
      PENABLE = 1;
      @(negedge PCLK);
      $display("APB READ Addr=0x%0x: PRDATA=0x%02h", addr, PRDATA);
      PSEL    = 0;
      PENABLE = 0;
    end
  endtask

  // Main Simulation
  initial begin
    $display("=== STARTING SPI DEBUG TB: CPOL=1, CPHA=1, LSBFE=1 ===");
    PRESETn = 0;
    miso = 0;
    PWRITE = 0;
    PSEL   = 0;
    PENABLE = 0;
    PADDR = 0;
    PWDATA = 0;

    #20 PRESETn = 1;

    // Configure SPI via APB
    apb_write(3'b000, 8'b1101_0101); // CR1: CPHA=1, CPOL=1, LSBFE=1, MSTR=1, SPE=1
    apb_write(3'b001, 8'b0001_1011); // CR2: SSI=1, SSOE=1, TXEIE=1, RXNEIE=1
    apb_write(3'b010, 8'b0000_0010); // BR: SPPR=0, SPR=2 => BR = 8

    // Provide TX data via APB
    apb_write(3'b101, 8'b10101010);  // TX data

    // Force outputs and control flags only
    force DUT.spi_mode   = 2'b00;
    force DUT.sppr       = 3'd0;
    force DUT.spr        = 3'd2;
    force DUT.spiswai    = 1'b0;
    force DUT.ss         = 1'b0;
    force DUT.cpol       = 1'b1;
    force DUT.cpha       = 1'b1;

    force DUT.send_data     = 1;
    force DUT.receive_data  = 1;
    force DUT.data_mosi     = 8'b10101010;
    force DUT.lsbfe         = 1;
    force DUT.SHIFT_REG.temp_reg = 8'b10101010;

    // Simulate SCLK + Flags (BR=8 -> 80ns period, 40ns half-cycle)
    repeat (8) begin
      force DUT.sclk        = 1;
      force DUT.flag_high   = 1;
      force DUT.flags_high  = 1;
      #40;

      force DUT.sclk        = 0;
      force DUT.flag_high   = 0;
      force DUT.flags_high  = 0;
      force DUT.flag_low    = 1;
      force DUT.flags_low   = 1;
      #40;

      force DUT.flag_low    = 0;
      force DUT.flags_low   = 0;
    end

    // Read received data
    apb_read(3'b011);

    $finish;
  end

  // Monitor key signals
  initial begin
    $monitor("T=%0t | SCLK=%b | MOSI=%b | MISO=%b | SS=%b | PRDATA=0x%02h",
              $time, sclk, mosi, miso, ss, PRDATA);
  end

endmodule
