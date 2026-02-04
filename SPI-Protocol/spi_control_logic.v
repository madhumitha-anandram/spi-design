module spi_control_logic (
    input  wire        PCLK,        // Clock
    input  wire        PRESETn,     // Active-low reset
            
    input  wire        send_data,   // Control signal
    input  wire [1:0]  spi_mode,    // SPI mode
    input  wire        spiswai,     // SPI wait input
    input  wire        mstr,        // Master mode
           
    input  wire [11:0]  BaudRateDivisor,      // Target count
    output reg        receive_data,// Output from D flip-flop
    output wire        tip,        // Transfer in progress
    output reg         ss         
);
reg [15:0] count;
reg rcv; 

// Compute target from BaudRateDivisor * 16
wire [15:0] target = BaudRateDivisor << 4;

always @(posedge PCLK or negedge PRESETn) begin
        if(!PRESETn)
          receive_data = 1'b0;
    
     else 
        receive_data = rcv;

end
assign tip = ~ss;
wire count_match = (count == (target - 1'b1)) ? 1'b1 : rcv;
wire count_match1 = (count <= (target - 1'b1)) ? count_match : 1'b0;
wire next_mux = send_data ? 1'b0 : count_match1;
wire mode_valid = (spi_mode == 2'b00) || (spi_mode == 2'b01) & ~spiswai & mstr;
wire next_mux1 = mode_valid ? next_mux : 1'b0;

//slave select block

wire ss_count_match = (count <= (target - 1'b1)) ? 1'b0 : 1'b1;
wire ss_next_mux = send_data ? 1'b0 : ss_count_match;
wire ss_next_mux1 = mode_valid ? ss_next_mux : 1'b1;

//counter block 

wire [15:0] target_match = (count <= (target - 1'b1)) ? (count + 1'b1) : 16'hffff;
wire [15:0] target_match_mux = send_data ? 16'b0 : target_match;
wire [15:0] count_next_mux1 = mode_valid ? target_match_mux : 16'hffff;

always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            ss <= 1'b1;
            rcv <= 1'b0;
            count <= 16'hffff;
            end
        else begin
            ss <= ss_next_mux1;
            rcv <= next_mux1;
            count <= count_next_mux1;
          end
    end

endmodule



module spi_control_logic_tb;

    // DUT Inputs
    reg         PCLK;
    reg         PRESETn;
    reg         send_data;
    reg  [1:0]  spi_mode;
    reg         spiswai;
    reg         mstr;
    reg  [11:0] BaudRateDivisor;

    // DUT Outputs
    wire        receive_data;
    wire        tip;
    wire        ss;

    // Instantiate the DUT
    spi_control_logic uut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .send_data(send_data),
        .spi_mode(spi_mode),
        .spiswai(spiswai),
        .mstr(mstr),
        .BaudRateDivisor(BaudRateDivisor),
        .receive_data(receive_data),
        .tip(tip),
        .ss(ss)
    );

    // Clock Generation (10ns period = 100MHz)
    initial PCLK = 0;
    always #5 PCLK = ~PCLK;

    // Tasks
    task reset_dut;
        begin
            PRESETn = 0;
            #20;
            PRESETn = 1;
            #20;
        end
    endtask

    task configure_inputs(
        input [1:0] mode,
        input       wait_flag,
        input       master_flag,
        input [11:0] baud
    );
        begin
            spi_mode        = mode;
            spiswai         = wait_flag;
            mstr            = master_flag;
            BaudRateDivisor = baud;
        end
    endtask

    task stimulate_send;
        begin
            send_data = 1;
            @(posedge PCLK);
            send_data = 0;
        end
    endtask

    task simulate_cycles(input integer n);
        begin
            repeat (n) @(posedge PCLK);
        end
    endtask

    // Main Stimulus
    initial begin
        // Default init
        send_data = 0;
        spi_mode = 2'b00;
        spiswai = 0;
        mstr = 1;
        BaudRateDivisor = 12'd10;

        // Reset the DUT
        reset_dut;

        // Configure SPI control
        configure_inputs(2'b00, 0, 1, 12'd10);

        // Trigger a send
        stimulate_send;

        // Simulate for a while
        simulate_cycles(100);

        $finish;
    end

    // Output Monitoring
    initial begin
        $monitor("Time=%0t | ss=%b | tip=%b | receive_data=%b | count=%0d",
                 $time, ss, tip, receive_data, uut.count);
    end

endmodule
