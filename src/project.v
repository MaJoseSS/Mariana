module tt_um_uart (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // Asignación de pines
    wire RX_IN    = ui_in[0];
    wire TX_START = ui_in[1];
    wire BAUD_EN  = ui_in[2];
    wire CTRL0    = ui_in[3];
    wire CTRL1    = ui_in[4];
    wire CTRL2    = ui_in[5];
    wire CTRL3    = ui_in[6];
    wire CTRL4    = ui_in[7];
    
    // Configuración
    wire [1:0] data_bits = {CTRL1, CTRL0};
    wire       parity_en = ~CTRL3;
    wire       parity_sel = CTRL2;
    wire       stop_bits = CTRL4;
    
    // Salidas principales
    wire TX_OUT, TX_BUSY, RX_READY, RX_ERROR;
    
    assign uo_out[0] = TX_OUT;
    assign uo_out[1] = TX_BUSY;
    assign uo_out[2] = RX_READY;
    assign uo_out[3] = RX_ERROR;
    assign uo_out[7:4] = 4'b0;
    
    // Bidireccionales
    assign uio_out = 8'b0;
    assign uio_oe = 8'b00000000;
    
    // Instancia UART
    uart uart_inst (
        .clk(clk),
        .rst(!rst_n),
        .rx_in(RX_IN),
        .tx_start(TX_START),
        .baud_en(BAUD_EN),
        .data_in(uio_in),
        .data_bits(data_bits),
        .parity_en(parity_en),
        .parity_sel(parity_sel),
        .stop_bits(stop_bits),
        .tx_out(TX_OUT),
        .tx_busy(TX_BUSY),
        .rx_ready(RX_READY),
        .rx_error(RX_ERROR)
    );

endmodule

module uart (
    input        clk,
    input        rst,
    input        rx_in,
    input        tx_start,
    input        baud_en,
    input  [7:0] data_in,
    input  [1:0] data_bits,
    input        parity_en,
    input        parity_sel,
    input        stop_bits,
    output       tx_out,
    output       tx_busy,
    output       rx_ready,
    output       rx_error
);

    // Estados TX
    typedef enum {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_PARITY,
        TX_STOP1,
        TX_STOP2
    } tx_state_t;
    
    // Estados RX
    typedef enum {
        RX_IDLE,
        RX_START,
        RX_DATA,
        RX_PARITY,
        RX_STOP1,
        RX_STOP2
    } rx_state_t;

    // Registros TX
    reg [2:0] tx_state;
    reg [2:0] tx_bit_cnt;
    reg [7:0] tx_data;
    reg       tx_reg;
    reg       tx_busy_reg;
    
    // Registros RX
    reg [2:0] rx_state;
    reg [2:0] rx_bit_cnt;
    reg [7:0] rx_data;
    reg       rx_ready_reg;
    reg       rx_error_reg;
    reg [3:0] rx_sample_cnt;
    reg [2:0] rx_sync;
    reg       rx_sample;
    
    // Configuración bits
    wire [2:0] num_bits = 5 + data_bits;

    // Sincronización RX
    always @(posedge clk or posedge rst) begin
        if (rst) rx_sync <= 3'b111;
        else rx_sync <= {rx_sync[1:0], rx_in};
    end
    
    // Detección de flanco
    wire rx_falling_edge = (rx_sync[2:1] == 2'b10);
    
    // Muestreo estabilizado
    always @(posedge clk) begin
        case (rx_sync[2:0])
            3'b000, 3'b001, 3'b010, 3'b100: rx_sample <= 1'b0;
            default: rx_sample <= 1'b1;
        endcase
    end

    // Cálculo de paridad (fuera del always)
    wire tx_parity_bit = parity_sel ? 
        ~(^tx_data[num_bits-1:0]) : 
         (^tx_data[num_bits-1:0]);
    
    // Máquina de estados TX
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_state <= TX_IDLE;
            tx_reg <= 1'b1;
            tx_busy_reg <= 1'b0;
        end
        else if (baud_en) begin
            case (tx_state)
                TX_IDLE: begin
                    tx_reg <= 1'b1;
                    if (tx_start && !tx_busy_reg) begin
                        tx_data <= data_in;
                        tx_busy_reg <= 1'b1;
                        tx_state <= TX_START;
                    end
                end
                
                TX_START: begin
                    tx_reg <= 1'b0;
                    tx_bit_cnt <= 3'd0;
                    tx_state <= TX_DATA;
                end
                
                TX_DATA: begin
                    tx_reg <= tx_data[tx_bit_cnt];
                    if (tx_bit_cnt == num_bits - 1) begin
                        if (parity_en) tx_state <= TX_PARITY;
                        else tx_state <= TX_STOP1;
                    end
                    else tx_bit_cnt <= tx_bit_cnt + 1;
                end
                
                TX_PARITY: begin
                    tx_reg <= tx_parity_bit;
                    tx_state <= TX_STOP1;
                end
                
                TX_STOP1: begin
                    tx_reg <= 1'b1;
                    if (stop_bits) tx_state <= TX_STOP2;
                    else begin
                        tx_busy_reg <= 1'b0;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_STOP2: begin
                    tx_reg <= 1'b1;
                    tx_busy_reg <= 1'b0;
                    tx_state <= TX_IDLE;
                end
            endcase
        end
    end

    assign tx_out = tx_reg;
    assign tx_busy = tx_busy_reg;

    // Cálculo de paridad RX (fuera del always)
    wire rx_base_parity = ^rx_data[num_bits-1:0];
    wire rx_exp_parity = parity_sel ? ~rx_base_parity : rx_base_parity;
    
    // Máquina de estados RX
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_state <= RX_IDLE;
            rx_ready_reg <= 1'b0;
            rx_error_reg <= 1'b0;
            rx_sample_cnt <= 4'd0;
        end
        else if (baud_en) begin
            rx_ready_reg <= 1'b0;
            
            case (rx_state)
                RX_IDLE: begin
                    rx_error_reg <= 1'b0;
                    rx_sample_cnt <= 4'd0;
                    if (rx_falling_edge) begin
                        rx_state <= RX_START;
                    end
                end
                
                RX_START: begin
                    if (rx_sample_cnt == 4'd7) begin
                        if (rx_sample) begin
                            rx_state <= RX_IDLE;
                            rx_error_reg <= 1'b1;
                        end
                    end
                    else if (rx_sample_cnt == 4'd15) begin
                        rx_state <= RX_DATA;
                        rx_bit_cnt <= 3'd0;
                        rx_sample_cnt <= 4'd0;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_DATA: begin
                    if (rx_sample_cnt == 4'd7) begin
                        rx_data[rx_bit_cnt] <= rx_sample;
                    end
                    else if (rx_sample_cnt == 4'd15) begin
                        if (rx_bit_cnt == num_bits - 1) begin
                            if (parity_en) rx_state <= RX_PARITY;
                            else rx_state <= RX_STOP1;
                        end
                        else rx_bit_cnt <= rx_bit_cnt + 1;
                        rx_sample_cnt <= 4'd0;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_PARITY: begin
                    if (rx_sample_cnt == 4'd7) begin
                        if (rx_sample != rx_exp_parity)
                            rx_error_reg <= 1'b1;
                    end
                    else if (rx_sample_cnt == 4'd15) begin
                        rx_state <= RX_STOP1;
                        rx_sample_cnt <= 4'd0;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_STOP1: begin
                    if (rx_sample_cnt == 4'd7) begin
                        if (!rx_sample)
                            rx_error_reg <= 1'b1;
                    end
                    else if (rx_sample_cnt == 4'd15) begin
                        if (stop_bits) begin
                            rx_state <= RX_STOP2;
                            rx_sample_cnt <= 4'd0;
                        end
                        else begin
                            rx_ready_reg <= 1'b1;
                            rx_state <= RX_IDLE;
                        end
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_STOP2: begin
                    if (rx_sample_cnt == 4'd7) begin
                        if (!rx_sample)
                            rx_error_reg <= 1'b1;
                    end
                    else if (rx_sample_cnt == 4'd15) begin
                        rx_ready_reg <= 1'b1;
                        rx_state <= RX_IDLE;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
            endcase
        end
    end

    assign rx_ready = rx_ready_reg;
    assign rx_error = rx_error_reg;

endmodule
