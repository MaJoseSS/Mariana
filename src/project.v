module tt_um_uart (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // Bidirectional data (inputs only)
    output wire [7:0] uio_out,  // Bidirectional data (unused)
    output wire [7:0] uio_oe,   // Bidirectional enable (always input)
    input  wire       ena,      // unused
    input  wire       clk,      // clock
    input  wire       rst_n     // reset (active low)
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
    wire [1:0] data_bits = {CTRL1, CTRL0}; // 00=5b, 01=6b, 10=7b, 11=8b
    wire       parity_en = ~CTRL3;         // 0=habilitada
    wire       parity_sel = CTRL2;         // 0=impar, 1=par
    wire       stop_bits = CTRL4;          // 0=1bit, 1=2bits
    
    // Salidas principales
    wire TX_OUT, TX_BUSY, RX_READY, RX_ERROR;
    
    assign uo_out[0] = TX_OUT;    // Salida serial
    assign uo_out[1] = TX_BUSY;   // Transmisor ocupado
    assign uo_out[2] = RX_READY;  // Dato recibido listo
    assign uo_out[3] = RX_ERROR;  // Error de recepción
    assign uo_out[7:4] = 4'b0;    // No utilizados
    
    // Bidireccionales (solo entrada)
    assign uio_out = 8'b0;
    assign uio_oe = 8'b00000000;  // Siempre en modo entrada
    
    // Instancia del UART configurable
    uart uart_inst (
        .clk(clk),
        .rst(!rst_n),         // Reset activo alto
        .rx_in(RX_IN),
        .tx_start(TX_START),
        .baud_en(BAUD_EN),
        .data_in(uio_in),     // Datos de entrada bidireccionales
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

// ======================= MÓDULO UART CONFIGURABLE =======================
module uart (
    input        clk,
    input        rst,
    input        rx_in,
    input        tx_start,
    input        baud_en,
    input  [7:0] data_in,
    input  [1:0] data_bits,   // Config bits de datos
    input        parity_en,   // Habilitación paridad
    input        parity_sel,  // 0=impar, 1=par
    input        stop_bits,   // Bits de stop
    output       tx_out,
    output       tx_busy,
    output       rx_ready,
    output       rx_error
);

    // Parámetros de estado
    typedef enum {
        TX_IDLE,
        TX_START,
        TX_DATA,
        TX_PARITY,
        TX_STOP1,
        TX_STOP2
    } tx_state_t;
    
    typedef enum {
        RX_IDLE,
        RX_START,
        RX_DATA,
        RX_PARITY,
        RX_STOP1,
        RX_STOP2
    } rx_state_t;

    // ======================= TRANSMISOR =======================
    reg [2:0] tx_state, tx_next;
    reg [2:0] tx_bit_cnt;
    reg [7:0] tx_data;
    reg       tx_reg;
    reg       tx_busy_reg;
    reg [3:0] tx_sample_cnt;  // Contador 16x
    
    // Configuración bits
    wire [2:0] num_bits = 5 + data_bits;  // 5-8 bits
    
    // Cálculo paridad
    wire parity_bit = parity_sel ? 
        ~(^tx_data[num_bits-1:0]) :  // Par
         (^tx_data[num_bits-1:0]);   // Impar
    
    // Máquina de estados TX
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_state <= TX_IDLE;
            tx_reg <= 1'b1;
            tx_busy_reg <= 1'b0;
            tx_sample_cnt <= 4'd0;
        end
        else if (baud_en) begin
            tx_state <= tx_next;
            
            case (tx_state)
                TX_IDLE: begin
                    tx_reg <= 1'b1;
                    if (tx_start && !tx_busy_reg) begin
                        tx_data <= data_in;
                        tx_busy_reg <= 1'b1;
                        tx_next <= TX_START;
                    end
                end
                
                TX_START: begin
                    tx_reg <= 1'b0;
                    tx_bit_cnt <= 3'd0;
                    tx_next <= TX_DATA;
                end
                
                TX_DATA: begin
                    tx_reg <= tx_data[tx_bit_cnt];
                    if (tx_bit_cnt == num_bits - 1) begin
                        if (parity_en) tx_next <= TX_PARITY;
                        else tx_next <= TX_STOP1;
                    end
                    else tx_bit_cnt <= tx_bit_cnt + 1;
                end
                
                TX_PARITY: begin
                    tx_reg <= parity_bit;
                    tx_next <= TX_STOP1;
                end
                
                TX_STOP1: begin
                    tx_reg <= 1'b1;
                    if (stop_bits) tx_next <= TX_STOP2;
                    else begin
                        tx_busy_reg <= 1'b0;
                        tx_next <= TX_IDLE;
                    end
                end
                
                TX_STOP2: begin
                    tx_reg <= 1'b1;
                    tx_busy_reg <= 1'b0;
                    tx_next <= TX_IDLE;
                end
            endcase
        end
    end

    assign tx_out = tx_reg;
    assign tx_busy = tx_busy_reg;

    // ======================= RECEPTOR =======================
    reg [2:0] rx_state, rx_next;
    reg [2:0] rx_bit_cnt;
    reg [7:0] rx_data;
    reg       rx_ready_reg;
    reg       rx_error_reg;
    reg [3:0] rx_sample_cnt;  // Contador 16x
    reg [3:0] rx_samples;     // Muestras para detección
    
    // Máquina de estados RX
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_state <= RX_IDLE;
            rx_ready_reg <= 1'b0;
            rx_error_reg <= 1'b0;
            rx_sample_cnt <= 4'd0;
        end
        else if (baud_en) begin
            rx_state <= rx_next;
            rx_ready_reg <= 1'b0;
            
            // Muestreo 16x en bit medio
            if (rx_sample_cnt == 4'd7) 
                rx_samples <= {rx_samples[2:0], rx_in};
            
            case (rx_state)
                RX_IDLE: begin
                    rx_error_reg <= 1'b0;
                    if (!rx_in) begin  // Detección start bit
                        rx_sample_cnt <= 4'd0;
                        rx_next <= RX_START;
                    end
                end
                
                RX_START: begin
                    if (rx_sample_cnt == 4'd15) begin
                        if (!rx_samples[2]) begin // Verificar start bit
                            rx_bit_cnt <= 3'd0;
                            rx_next <= RX_DATA;
                        end
                        else rx_next <= RX_IDLE;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_DATA: begin
                    if (rx_sample_cnt == 4'd15) begin
                        rx_data[rx_bit_cnt] <= rx_samples[2]; // Bit medio
                        if (rx_bit_cnt == num_bits - 1) begin
                            if (parity_en) rx_next <= RX_PARITY;
                            else rx_next <= RX_STOP1;
                        end
                        else rx_bit_cnt <= rx_bit_cnt + 1;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_PARITY: begin
                    if (rx_sample_cnt == 4'd15) begin
                        // Verificar paridad
                        wire exp_parity = parity_sel ? 
                            ~(^rx_data[num_bits-1:0]) : 
                             (^rx_data[num_bits-1:0]);
                        if (rx_samples[2] != exp_parity)
                            rx_error_reg <= 1'b1;
                        rx_next <= RX_STOP1;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_STOP1: begin
                    if (rx_sample_cnt == 4'd15) begin
                        if (rx_samples[2] !== 1'b1) // Error framing
                            rx_error_reg <= 1'b1;
                        if (stop_bits) 
                            rx_next <= RX_STOP2;
                        else begin
                            rx_ready_reg <= 1'b1;
                            rx_next <= RX_IDLE;
                        end
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
                
                RX_STOP2: begin
                    if (rx_sample_cnt == 4'd15) begin
                        if (rx_samples[2] !== 1'b1) // Error framing
                            rx_error_reg <= 1'b1;
                        rx_ready_reg <= 1'b1;
                        rx_next <= RX_IDLE;
                    end
                    else rx_sample_cnt <= rx_sample_cnt + 1;
                end
            endcase
        end
    end

    assign rx_ready = rx_ready_reg;
    assign rx_error = rx_error_reg;

endmodule
