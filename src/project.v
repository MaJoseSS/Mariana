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

// Primero el modulo de tx 

module tx(
  input reinicio,
  input clock,
  input [7:0] info_in,
  input start,
  output reg TX,
  output reg ocupado);
  

  typedef enum reg [1:0] {Reposo, Start, Info, Stop} Estados;
 
  Estados EstadosTx;
  reg [2:0] Indexes;
  reg [7:0] DataSaved;

  
  always @(posedge clock or 
           posedge reinicio) begin
    if (reinicio) begin
    	EstadosTx <= Reposo;
      Indexes <= 0; TX <= 1; ocupado <= 0;
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
          if (Indexes == 7) EstadosTx <= Stop;
          else Indexes <= Indexes +1;
        end
        
        Stop: begin
          TX <= 1;
          EstadosTx <= Reposo;
        end
        endcase
      	end
        end
     endmodule
       
// Seccion del rx
       
module rx (
  input RX,
  input clock,
  input reinicio,
  output reg Terminado,
  output reg [7:0] info_out);
  reg WaitRx;
 
  typedef enum reg [1:0] {Reposo, Start, Info, Stop} Estados;
  
  Estados EstadosRx;
  reg [2:0] Indexes;
  reg [7:0] DataSaved;
  
  
always @(posedge clock or posedge reinicio) begin
  if (reinicio)
    WaitRx <= 1'b1;  
  else
    WaitRx <= RX;    
end
  
  
  always @(posedge clock or posedge reinicio) begin 
    if (reinicio) begin
      EstadosRx <= Reposo; 
      Terminado <= 0;
      Indexes <= 0;
    end
    else begin
      case (EstadosRx)
        Reposo: begin
          Terminado <= 0;
          if (RX == 0) begin
            EstadosRx <= Start;
          end
        end
       	Start: begin
           EstadosRx <= Info;
           Indexes <= 0;
         end
        Info: begin
          DataSaved[Indexes] <= WaitRx;
           if (Indexes == 7)begin
             EstadosRx <= Stop;
           end
           else Indexes <= Indexes +1;
         end
         
         Stop: begin
            
           info_out <= DataSaved;
           Terminado <=1;
           EstadosRx <= Reposo;
          end
          endcase
        end
        end
        endmodule 
  
// Juntando ambas
          
module uart_inst (
  input clockUart,
  input resetUart,
  input StartUart,
  input [7:0] info_in_Uart,
  output [7:0] info_out_Uart,
  output TerminadoUart);
  
  wire Tx;
  
  tx txUart (
    .clock(clockUart),
    .reinicio(resetUart),
    .info_in(info_in_Uart),
    .start(StartUart),
    .TX(Tx),
    .ocupado());
  
  rx rxUart (
    .clock(clockUart),
    .reinicio(resetUart),
    .RX(Tx),
    .info_out(info_out_Uart),
    .Terminado(TerminadoUart));
  
endmodule
            
  
