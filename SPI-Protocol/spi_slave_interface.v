module spi_slave_interface (
  input        PCLK,
  input        PRESETn,
  input  [2:0] PADDR,
  input        PWRITE,
  input        PSEL,
  input        PENABLE,
  input  [7:0] PWDATA,
  input        ss,
  input  [7:0] data_miso,
  input        receive_data,
  input        tip,
  output [7:0] PRDATA,
  output       mstr,
  output       cpol,
  output       cpha,
  output       lsbfe,
  output       spiswai,
  output       spi_interrupt_request,
  output       PREADY,
  output       PSLVERR,
  output reg   send_data,
  output [7:0] data_mosi,
  output reg [1:0] spi_mode,
  output  [2:0]     sppr,
  output  [2:0]     spr);
  wire spif, sptef, modf;
  // FSM State Parameters
  parameter IDLE   = 2'b00;
  parameter SETUP  = 2'b01;
  parameter ENABLE = 2'b10;

  reg [1:0] state, next_state;

  // SPI Modes
  parameter SPI_RUN  = 2'b00;
  parameter SPI_WAIT = 2'b01;
  parameter SPI_STOP = 2'b10;

 reg [1:0] next_spi_mode;

  // Address Parameters
  parameter ADDR_CR1  = 3'b000;
  parameter ADDR_CR2  = 3'b001;
  parameter ADDR_BR   = 3'b010;
  parameter ADDR_DR_R = 3'b011;
  parameter ADDR_DR_W = 3'b101;

  // Registers
  reg [7:0] SPI_CR_1, SPI_CR_2, SPI_BR, SPI_DR;
  wire [7:0] SPI_SR;
  
  // Control Bit Decoding
  assign mstr     = SPI_CR_1[4];
  assign cpol     = SPI_CR_1[3];
  assign cpha     = SPI_CR_1[2];
  assign lsbfe    = SPI_CR_1[0];
  wire spie     = SPI_CR_1[7];
  wire spe      = SPI_CR_1[6];
  wire sptie    = SPI_CR_1[5];
  wire modfen   = SPI_CR_2[4];
  assign spiswai  = SPI_CR_2[1];
  assign  spr  = SPI_BR[2:0];
  assign  sppr = SPI_BR[6:4];

assign modf  = ~ss & mstr & modfen& ~spie;
assign SPI_SR = !PRESETn? 8'b00100000: {spif, 1'b0, sptef, modf, 4'b0000};
assign sptef = (SPI_DR == 8'h00)?1'b1:1'b0;
assign spif  = (SPI_DR != 8'h00)?1'b1:1'b0;

  // FSM State Register
  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      state <= IDLE;
    else
      state <= next_state;
  end

  // FSM Next State Logic
  always @(*) begin
    case (state)
      IDLE:   next_state = (PSEL && !PENABLE) ? SETUP : IDLE;
      SETUP:  next_state = (PSEL && PENABLE) ? ENABLE : IDLE;
      SETUP:  next_state = (PSEL && !PENABLE) ? SETUP : IDLE;
      ENABLE: next_state = PSEL ? SETUP : IDLE;
      default: next_state = IDLE;
    endcase
  end
 
 // State Register
always @(posedge PCLK or negedge PRESETn) begin
  if (!PRESETn)
    spi_mode <= SPI_RUN;
  else
    spi_mode <= next_spi_mode;
end

// Next State Logic (your format)
always @(*) begin
  case (spi_mode)
    SPI_STOP: begin
      if (spe)
        next_spi_mode = SPI_RUN;
      else if(!spiswai)
        next_spi_mode = SPI_WAIT;
      else
        next_spi_mode = SPI_STOP;
    end

    SPI_RUN: begin
      if (!spe)
        next_spi_mode = SPI_WAIT;
      else
        next_spi_mode = SPI_RUN;
    end

    SPI_WAIT: begin
      if (spiswai)
        next_spi_mode = SPI_STOP;
      else if (spe)
        next_spi_mode = SPI_RUN;
      else
        next_spi_mode = SPI_WAIT;
    end

    default: next_spi_mode = SPI_RUN;
  endcase
end

  // APB Outputs
  assign PREADY  = (state == ENABLE)? 1'b1:1'b0;
  assign PSLVERR = (state == ENABLE)? tip:1'b0;

  // Write / Read Enable
  wire wr_enb = ((state == ENABLE) & PWRITE)? 1'b1 : 1'b0;
  wire rd_enb = ((state == ENABLE) & ~PWRITE)? 1'b1 : 1'b0;
 
wire [7:0] cr2_mask = 8'b0001_1011;
wire [7:0] br_mask = 8'b0111_0111;
  // Register Write Logic
  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      SPI_CR_1 <= 8'h00;
    else if (wr_enb && PADDR == ADDR_CR1)
      SPI_CR_1 <= PWDATA;
    
  end

  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      SPI_CR_2 <= 8'h00;
    else if (wr_enb && PADDR == ADDR_CR2)
      SPI_CR_2 <= (PWDATA & cr2_mask);
    
  end

  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      SPI_BR <= 8'h00;
    else if (wr_enb && PADDR == ADDR_BR)
      SPI_BR <= (PWDATA & br_mask);
    
  end

wire cond1, cond2, cond3, cond4,cond5,cond6, cond7;
wire all_conditions;

// Condition 1: SPI_DR != iso_data
assign cond1 = (SPI_DR != data_miso);

// Condition 2: spi_mode == SPI_RUN || spi_mode == SPI_WAIT
assign cond2 = (spi_mode == SPI_RUN) || (spi_mode == SPI_WAIT);

// Condition 3: SPI_DR != 0
assign cond3 = (SPI_DR == PWDATA);

// Combine all with receive_data input (4-input AND gate)
assign all_conditions = cond1 & cond2 & cond3;

assign cond4 = receive_data&cond2;
assign cond5 = cond4? 1'b0:1'b1;
assign cond6 = all_conditions? 1'b1:cond5;
assign cond7 = wr_enb? send_data:cond6;

// Output logic for send_data
always @(posedge PCLK or negedge PRESETn) begin
  if (!PRESETn)
    send_data <= 1'b0;
  else
    send_data <= cond7;
end

// SPI_DR block

wire [7:0] cond8 = cond4 ? data_miso:SPI_DR;
wire [7:0] cond9 = all_conditions? 8'b0:SPI_DR;
wire [7:0] cond10 = (PADDR==3'b101)? PWDATA : cond8;
wire [7:0] cond11 = wr_enb?cond10:cond9;

always @(posedge PCLK or negedge PRESETn) begin
  if (!PRESETn)
    SPI_DR <= 8'b0;
  else
    SPI_DR <= cond11;
end  
  
reg [7:0] temp_reg;
reg [2:0] bit_cnt;

always @(posedge PCLK or negedge PRESETn) begin
  if (!PRESETn) begin
    temp_reg <= 8'b0;
    bit_cnt  <= 3'd0;
  end
  else if (send_data) begin
    temp_reg <= SPI_DR;     // Load data on send_data trigger
    bit_cnt  <= 3'd0;        // Reset bit count
  end
  else if (bit_cnt < 3'd7) begin
    temp_reg <= lsbfe ? {1'b0, temp_reg[7:1]} : {temp_reg[6:0], 1'b0};
    bit_cnt  <= bit_cnt + 1;
  end
end

assign data_mosi = lsbfe ? temp_reg[0] : temp_reg[7];


  // Read Logic
wire [7:0] prdata_mux1_out;
wire [7:0] prdata_mux2_out;
wire [7:0] prdata_mux3_out;
wire [7:0] prdata_mux4_out;

assign prdata_mux3_out = (PADDR == 3'b011) ? SPI_DR : SPI_SR;
assign prdata_mux2_out = (PADDR == 3'b010) ? SPI_BR : prdata_mux3_out;
assign prdata_mux1_out = (PADDR == 3'b001) ? SPI_CR_2 : prdata_mux2_out;
assign prdata_mux4_out = (PADDR == 3'b000) ? SPI_CR_1 : prdata_mux1_out;
assign PRDATA = rd_enb? prdata_mux4_out: 8'b0;

// Interrupt Logic
  wire modf_or_spif = modf | spif|sptef;
  wire logic1 = (~spie & sptie)? sptef : modf_or_spif;
  wire logic2 = (~sptie&spie)? (spif|modf) : logic1;
  assign spi_interrupt_request = (~spie & ~sptie)? 1'b0: logic2 ;
endmodule



module spi_slave_interface_tb;

  reg        PCLK;
  reg        PRESETn;
  reg  [2:0] PADDR;
  reg        PWRITE;
  reg        PSEL;
  reg        PENABLE;
  reg  [7:0] PWDATA;
  reg        ss;
  reg  [7:0] data_miso;
  reg        receive_data;
  reg        tip;

  wire [7:0] PRDATA;
  wire       mstr, cpol, cpha, lsbfe, spiswai;
  wire       spi_interrupt_request;
  wire       PREADY, PSLVERR;
  wire [7:0] data_mosi;
  wire [1:0] spi_mode;
  wire [2:0] sppr, spr;
  wire       send_data;

  // Instantiate DUT
  spi_slave_interface uut (
    .PCLK(PCLK),
    .PRESETn(PRESETn),
    .PADDR(PADDR),
    .PWRITE(PWRITE),
    .PSEL(PSEL),
    .PENABLE(PENABLE),
    .PWDATA(PWDATA),
    .ss(ss),
    .data_miso(data_miso),
    .receive_data(receive_data),
    .tip(tip),
    .PRDATA(PRDATA),
    .mstr(mstr),
    .cpol(cpol),
    .cpha(cpha),
    .lsbfe(lsbfe),
    .spiswai(spiswai),
    .spi_interrupt_request(spi_interrupt_request),
    .PREADY(PREADY),
    .PSLVERR(PSLVERR),
    .send_data(send_data),
    .data_mosi(data_mosi),
    .spi_mode(spi_mode),
    .sppr(sppr),
    .spr(spr)
  );

  // Clock
  initial PCLK = 0;
  always #5 PCLK = ~PCLK;

 
  // APB Write Task
  task apb_write;
    input [2:0] addr;
    input [7:0] data;
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

  // APB Read Task
  task apb_read;
    input [2:0] addr;
    begin
      @(negedge PCLK);
      PADDR   = addr;
      PWRITE  = 0;
      PSEL    = 1;
      PENABLE = 0;
      @(negedge PCLK);
      PENABLE = 1;
      @(negedge PCLK);
      $display("READ [Addr=%0d]: PRDATA = 0x%02h, PSLVERR = %b", addr, PRDATA, PSLVERR);
      PSEL    = 0;
      PENABLE = 0;
    end
  endtask

  initial begin
    // Initialization
    PRESETn      = 0;
    PADDR        = 3'b000;
    PWRITE       = 0;
    PSEL         = 0;
    PENABLE      = 0;
    PWDATA       = 8'h00;
    ss           = 1;
    data_miso    = 8'h00;
    receive_data = 0;
    tip          = 0;

    #20 PRESETn = 1;

    // Write control registers
    apb_write(3'b000, 8'b1101_0101); // CR1: SPE, MSTR, CPOL, CPHA, LSBFE
    apb_write(3'b001, 8'b0001_1011); // CR2: MODFEN, SWAI
    apb_write(3'b010, 8'b0001_0111); // BR: SPPR=1, SPR=111 => usable baud rate

    // Write a value to SPI_DR (DATA REGISTER)
    apb_write(3'b101, 8'hBE); // Write 0xBE to SPI_DR

    // Wait a bit before reading
    #10;

    // Read back SPI_DR to check PRDATA
    apb_read(3'b011);  // DR_R (should show 0xBE)

    // Read CR1 and CR2 just for visibility
    apb_read(3'b000);  // CR1
    apb_read(3'b001);  // CR2

    // Simulate transfer complete by toggling send_data and receive_data
    @(negedge PCLK);
    receive_data = 1;
    data_miso = 8'hA5; // incoming data
    ss = 0;            // select slave

    @(negedge PCLK);
    receive_data = 0;
    ss = 1;

    // Read SPI_DR again (should now contain 0xA5)
    #10;
    apb_read(3'b011);  // DR_R again

    #50;
    $finish;
  end

endmodule
