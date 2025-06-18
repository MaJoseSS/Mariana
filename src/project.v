module tt_um_uart (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // Enable (ignored)
    input  wire       clk,      // Clock
    input  wire       rst_n     // Reset active low
);

// ================================================
// Reset conversion and signal mapping
// ================================================
wire rst = ~rst_n;  // Convert to active-high reset
wire rx_in = ui_in[0];
wire tx_start = ui_in[1];
wire [4:0] ctrl_word = ui_in[7:3]; // Unused in this implementation
wire [7:0] tx_data = uio_in;

// Internal signals
wire tx_out;
wire tx_busy;
wire [7:0] rx_data;
wire rx_ready;

// Output assignments
assign uo_out[0] = tx_out;    // Serial output
assign uo_out[1] = tx_busy;   // Transmitter busy
assign uo_out[2] = rx_ready;  // Data ready pulse
assign uo_out[3] = 1'b0;      // Error indicator (not used)
assign uo_out[7:4] = 4'b0;    // Unused outputs

assign uio_out = 8'b0;        // Not used
assign uio_oe = 8'h00;        // Always input mode

// ================================================
// UART Core Implementation (Simple version)
// ================================================
uart_core uart_inst (
    .clk(clk),
    .rst(rst),
    .ctrl_word(ctrl_word),    // Unused
    .tx_data(tx_data),
    .tx_start(tx_start),
    .tx_busy(tx_busy),
    .tx_out(tx_out),
    .rx_in(rx_in),
    .rx_data(rx_data),
    .rx_ready(rx_ready),
    .rx_error(),              // Not used
    .baud16_en(1'b1)         // Always enabled
);

endmodule

module uart_core (
    input clk,
    input rst,
    input [4:0] ctrl_word,    // Unused in this simple implementation
    input [7:0] tx_data,
    input tx_start,
    output tx_busy,
    output tx_out,
    input rx_in,
    output [7:0] rx_data,
    output rx_ready,
    output rx_error,          // Not implemented
    input baud16_en           // Unused
);

// ================================================
// Transmitter Module (Adapted from codigo1)
// ================================================
tx tx_inst (
    .reinicio(rst),
    .clock(clk),
    .info_in(tx_data),
    .start(tx_start),
    .TX(tx_out),
    .ocupado(tx_busy)
);

// ================================================
// Receiver Module (Adapted from codigo1)
// ================================================
rx rx_inst (
    .RX(rx_in),
    .clock(clk),
    .reinicio(rst),
    .Terminado(rx_ready),
    .info_out(rx_data)
);

// Error detection not implemented
assign rx_error = 1'b0;

endmodule

// ================================================
// Transmitter Submodule (Original functionality)
// ================================================
module tx(
    input reinicio,
    input clock,
    input [7:0] info_in,
    input start,
    output reg TX,
    output reg ocupado
);

    typedef enum reg [1:0] {Reposo, Start, Info, Stop} Estados;
    Estados EstadosTx;
    reg [2:0] Indexes;
    reg [7:0] DataSaved;

    always @(posedge clock or posedge reinicio) begin
        if (reinicio) begin
            EstadosTx <= Reposo;
            Indexes <= 0; 
            TX <= 1; 
            ocupado <= 0;
        end
        else begin
            case(EstadosTx)
                Reposo: begin
                    TX <= 1;
                    ocupado <= 0;
                    if (start) begin 
                        DataSaved <= info_in;
                        EstadosTx <= Start;
                        ocupado <= 1;
                    end
                end
                
                Start: begin
                    TX <= 0;
                    EstadosTx <= Info;
                    Indexes <= 0;
                end
                
                Info: begin
                    TX <= DataSaved[Indexes];
                    if (Indexes == 7) 
                        EstadosTx <= Stop;
                    else 
                        Indexes <= Indexes + 1;
                end
                
                Stop: begin
                    TX <= 1;
                    EstadosTx <= Reposo;
                end
            endcase
        end
    end
endmodule

// ================================================
// Receiver Submodule (Original functionality)
// ================================================
module rx (
    input RX,
    input clock,
    input reinicio,
    output reg Terminado,
    output reg [7:0] info_out
);

    reg WaitRx;
    typedef enum reg [1:0] {Reposo, Start, Info, Stop} Estados;
    Estados EstadosRx;
    reg [2:0] Indexes;
    reg [7:0] DataSaved;

    // Synchronizer flip-flop
    always @(posedge clock or posedge reinicio) begin
        if (reinicio)
            WaitRx <= 1'b1;  
        else
            WaitRx <= RX;    
    end

    // Main state machine
    always @(posedge clock or posedge reinicio) begin 
        if (reinicio) begin
            EstadosRx <= Reposo; 
            Terminado <= 0;
            Indexes <= 0;
        end
        else begin
            Terminado <= 0; // Default value
            case (EstadosRx)
                Reposo: begin
                    if (WaitRx == 0) begin
                        EstadosRx <= Start;
                    end
                end
                
                Start: begin
                    EstadosRx <= Info;
                    Indexes <= 0;
                end
                
                Info: begin
                    DataSaved[Indexes] <= WaitRx;
                    if (Indexes == 7) begin
                        EstadosRx <= Stop;
                    end
                    else begin
                        Indexes <= Indexes + 1;
                    end
                end
                
                Stop: begin
                    info_out <= DataSaved;
                    Terminado <= 1;
                    EstadosRx <= Reposo;
                end
            endcase
        end
    end
endmodule
