module spi_core (
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire        receive_data,
    input  wire        send_data,
    input  wire [7:0]  data_mosi,
    input  wire        ss,
    input  wire        cpha,
    input  wire        cpol,
    input  wire        miso,
    input  wire        lsbfe,
    input  wire        flags_low,
    input  wire        flags_high,
    input  wire        flag_low,
    input  wire        flag_high,
    output wire [7:0]  data_miso,
    output reg         mosi
);

    reg [7:0] shift_register;
    reg [2:0] count;
    reg [7:0] temp_reg;
// TEMP_REG UPDATE
    wire mode_sel = (~cpha & cpol) | (cpha & ~cpol);
    wire [2:0] recv_index = lsbfe ? count : (3'd7 - count);
    wire in_bit = (mode_sel ? (flag_high ? miso : temp_reg[recv_index]) 
                            : (flag_low  ? miso : temp_reg[recv_index]));
    wire write_enable = (!ss) && ((mode_sel && flag_high) || (!mode_sel && flag_low));

    always @(posedge PCLK or negedge PRESETn)
        if (!PRESETn)
            temp_reg <= 8'b0;
        else if (write_enable)
            temp_reg[recv_index] <= in_bit;

    assign data_miso = receive_data ? temp_reg: 8'h00;

    wire [7:0] mux_out2 = send_data ? data_mosi : shift_register;

    always @(posedge PCLK or negedge PRESETn)
        if (!PRESETn)
            shift_register <= 8'b0;
        else
            shift_register <= mux_out2;

   
    wire [2:0] bit_index = lsbfe ? count : (3'd7 - count);
    wire mux0_out = flags_high ? shift_register[bit_index] : mosi;
    wire mux1_out = (count <= 3'd7) ? mux0_out : mosi;
    wire mux2_out = lsbfe ? mux1_out : mosi;
    wire [2:0] bit_index1 = lsbfe ? count : (3'd7 - count);
    wire mux3_out = flags_high ? shift_register[bit_index1] : mosi;
    wire mux4_out = (count >= 3'd0) ? mux3_out : mosi;
    wire mux5_out = mode_sel ? mux2_out : mux4_out;
    wire mux6_out = ss ? mosi : mux5_out;

    always @(posedge PCLK or negedge PRESETn)
        if (!PRESETn)
            mosi <= 1'b0;
        else
            mosi <= mux6_out;

    // COUNT UPDATE
    wire [2:0] count_plus_1  = count + 1;
    wire [2:0] count_minus_1 = count - 1;
    wire [2:0] mux1 = (count <= 3'd7) ? count_plus_1 : 3'd0;
    wire [2:0] mux2 = (count >= 3'd0) ? count_minus_1 : 3'd7;
    wire [2:0] mux1_high = lsbfe ? mux1 : mux2;
    wire [2:0] mux3 = (count <= 3'd7) ? count_plus_1 : 3'd0;
    wire [2:0] mux4 = (count >= 3'd0) ? count_minus_1 : 3'd7;
    wire [2:0] mux1_low  = lsbfe ? mux3 : mux4;
    wire [2:0] final_mux_out = mode_sel ? mux1_high : mux1_low;

    wire count_enable = (mode_sel && flags_high) || (!mode_sel && flags_low);
    wire [2:0] final_count = count_enable ? final_mux_out : count;

    always @(posedge PCLK or negedge PRESETn)
        if (!PRESETn)
            count <= 3'd0;
        else if (!ss)
            count <= final_count;

    

endmodule


module spi_core_tb;

    reg         PCLK;
    reg         PRESETn;
    reg         receive_data;
    reg         send_data;
    reg  [7:0]  data_mosi;
    reg         ss;
    reg         cpha;
    reg         cpol;
    reg         miso;
    reg         lsbfe;
    reg         flags_low;
    reg         flags_high;
    reg         flag_low;
    reg         flag_high;

    wire [7:0]  data_miso;
    wire        mosi;

    spi_core_teporary uut (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .receive_data(receive_data),
        .send_data(send_data),
        .data_mosi(data_mosi),
        .ss(ss),
        .cpha(cpha),
        .cpol(cpol),
        .miso(miso),
        .lsbfe(lsbfe),
        .data_miso(data_miso),
        .mosi(mosi),
        .flags_low(flags_low),
        .flags_high(flags_high),
        .flag_low(flag_low),
        .flag_high(flag_high)
    );

    initial PCLK = 0;
    always #5 PCLK = ~PCLK;

    task reset_dut();
        begin
            PRESETn = 0; #20;
            PRESETn = 1; #20;
        end
    endtask

    task set_config(input [7:0] data, input lsb, input CPHA, input CPOL);
        begin
            data_mosi = data;
            lsbfe = lsb;
            cpha = CPHA;
            cpol = CPOL;
        end
    endtask

    task simulate_bit(input miso_val);
        begin
            miso = miso_val;
            flag_high = 1; flags_high = 1; #10;
            flag_high = 0; flags_high = 0; #10;
            flag_low  = 1; flags_low  = 1; #10;
            flag_low  = 0; flags_low  = 0; #10;
        end
    endtask

    integer i;
    reg [7:0] miso_stream;

    task run_transfer(input [7:0] miso_bits);
        begin
            ss = 0;
            send_data = 1;
            receive_data = 1;
            for (i = 7; i >= 0; i = i - 1)
                simulate_bit(miso_bits[i]);
            send_data = 0;
            receive_data = 0;
            ss = 1;
        end
    endtask

    initial begin
        PRESETn = 0;
        receive_data = 0;
        send_data = 0;
        ss = 1;
        data_mosi = 8'h00;
        miso = 0;
        lsbfe = 0;
        cpha = 0;
        cpol = 0;
        flags_low = 0;
        flags_high = 0;
        flag_low = 0;
        flag_high = 0;

        reset_dut();

        $display("=== SPI MSB Mode ===");
        set_config(8'b10101010, 0, 0, 0);
        run_transfer(8'b11001100);
        #40;
        $display("TEMP_REG MSB: %b", uut.temp_reg);

        $display("=== SPI LSB Mode ===");
        set_config(8'b11001100, 1, 1, 1);
        run_transfer(8'b10101010);
        #40;
        $display("TEMP_REG LSB: %b", uut.temp_reg);

        $finish;
    end

endmodule
