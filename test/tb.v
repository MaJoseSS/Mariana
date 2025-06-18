`default_nettype none
`timescale 1ns / 1ps

module tb ();

  // Dump the signals to a VCD file
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs
  reg clk = 0;
  reg rst_n = 0;
  reg ena = 1;
  reg [7:0] ui_in = 0;
  reg [7:0] uio_in = 0;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Instantiate the UART module
  tt_um_uart tt_um_uart (
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // Clock generation (100 MHz)
  always #5 clk = ~clk;

  // Baud rate configuration
  parameter CLOCK_FREQ = 100_000_000;  // 100 MHz
  parameter BAUD_RATE = 115_200;
  localparam BIT_PERIOD = 1_000_000_000 / BAUD_RATE; // ns (integer division)
  integer baud_counter = 0;
  reg baud16_en = 0;
  
  // Baud enable generator
  always @(posedge clk) begin
    if (!rst_n) begin
      baud_counter <= 0;
      baud16_en <= 0;
    end else if (baud_counter >= (CLOCK_FREQ / (BAUD_RATE * 16)) - 1) begin
      baud_counter <= 0;
      baud16_en <= 1;
    end else begin
      baud_counter <= baud_counter + 1;
      baud16_en <= 0;
    end
  end

  // Variables for debugging (si necesitas acceso a señales internas)
  reg [7:0] received_data;
  reg rx_ready_prev = 0;

  // Capturar datos recibidos cuando rx_ready se activa
  always @(posedge clk) begin
    rx_ready_prev <= uo_out[2];
    if (uo_out[2] && !rx_ready_prev) begin
      // rx_ready just went high, capture the data
      received_data <= uio_out; // Asumiendo que los datos están en uio_out
    end
  end

  // Test procedure
  initial begin
    // Initialize
    rst_n = 0;
    ui_in = 0;
    uio_in = 0;
    #100;
    
    // Release reset
    rst_n = 1;
    #100;
    
    // Configure UART for 8N1
    ui_in[7:3] = 5'b01011;  // stop=0 (1-bit), parity_disabled=1, data_bits=8 (11)
    ui_in[2] = baud16_en;   // Connect baud enable
    
    // Test 1: Basic transmission
    $display("[TEST 1] Basic transmission");
    uio_in = 8'hA5;
    ui_in[1] = 1'b1;  // Activate tx_start
    #20;
    ui_in[1] = 1'b0;  // Deactivate tx_start
    
    // Wait for transmission to start
    wait (uo_out[1] == 1'b1);
    $display("  Transmission started");
    
    // Wait for transmission to complete
    wait (uo_out[1] == 1'b0);
    $display("  Transmission complete");
    
    // Test 2: Basic reception
    $display("\n[TEST 2] Basic reception");
    fork
      begin
        // Send byte 0x5A serially
        // Start bit
        ui_in[0] = 1'b0;
        #(BIT_PERIOD);
        
        // Data bits (LSB first) - 0x5A = 01011010
        ui_in[0] = 1'b0; // bit0
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit1
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit2
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit3
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit4
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit5
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit6
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit7
        #(BIT_PERIOD);
        
        // Stop bit
        ui_in[0] = 1'b1;
        #(BIT_PERIOD * 2);
      end
    join
    
    // Wait for reception to complete
    wait (uo_out[2] == 1'b1);
    if (received_data == 8'h5A) 
      $display("  Reception successful: 0x%h", received_data);
    else
      $display("  ERROR: Received 0x%h, expected 0x5A", received_data);
    
    #100; // Wait before next test
    
    // Test 3: Parity error detection
    $display("\n[TEST 3] Parity error detection");
    // Configure for parity checking
    ui_in[7:3] = 5'b00111;  // stop=0, parity_disabled=0, parity_type=1 (even), data_bits=8 (11)
    
    fork
      begin
        // Send byte with incorrect parity (0x00 should have parity=0 for even)
        // Start bit
        ui_in[0] = 1'b0;
        #(BIT_PERIOD);
        
        // Data bits (0x00 = 00000000)
        for (integer i = 0; i < 8; i = i + 1) begin
          ui_in[0] = 1'b0;
          #(BIT_PERIOD);
        end
        
        // Incorrect parity bit (1 instead of 0)
        ui_in[0] = 1'b1;
        #(BIT_PERIOD);
        
        // Stop bit
        ui_in[0] = 1'b1;
        #(BIT_PERIOD * 2);
      end
    join
    
    // Wait for reception to complete
    wait (uo_out[2] == 1'b1);
    if (uo_out[3] == 1'b1) 
      $display("  Parity error detected successfully");
    else
      $display("  ERROR: Parity error not detected");
    
    #100; // Wait before next test
    
    // Test 4: Frame error detection
    $display("\n[TEST 4] Frame error detection");
    // Configure for no parity
    ui_in[7:3] = 5'b01011;
    
    fork
      begin
        // Send byte with short stop bit
        // Start bit
        ui_in[0] = 1'b0;
        #(BIT_PERIOD);
        
        // Data bits (0x55 = 01010101)
        ui_in[0] = 1'b1; // bit0
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit1
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit2
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit3
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit4
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit5
        #(BIT_PERIOD);
        ui_in[0] = 1'b1; // bit6
        #(BIT_PERIOD);
        ui_in[0] = 1'b0; // bit7
        #(BIT_PERIOD);
        
        // Short stop bit (only half period)
        ui_in[0] = 1'b1;
        #(BIT_PERIOD / 2);
        
        // Next start bit (violates stop bit timing)
        ui_in[0] = 1'b0;
        #(BIT_PERIOD);
        
        // Complete the next byte to avoid hanging
        for (integer i = 0; i < 8; i = i + 1) begin
          ui_in[0] = 1'b0;
          #(BIT_PERIOD);
        end
        ui_in[0] = 1'b1;
        #(BIT_PERIOD);
      end
    join
    
    // Wait for reception to complete
    wait (uo_out[2] == 1'b1);
    if (uo_out[3] == 1'b1) 
      $display("  Frame error detected successfully");
    else
      $display("  ERROR: Frame error not detected");
    
    #100; // Wait before next test
    
    // Test 5: Busy signal during transmission
    $display("\n[TEST 5] Busy signal check");
    uio_in = 8'hFF;
    ui_in[1] = 1'b1;
    #20;
    ui_in[1] = 1'b0;
    
    // Check if busy signal becomes active
    #100; // Give some time for the transmission to start
    if (uo_out[1] === 1'b1)
      $display("  Busy signal active during transmission");
    else
      $display("  ERROR: Busy signal not active");
    
    // Wait for transmission to complete
    wait (uo_out[1] == 1'b0);
    $display("  Busy signal deactivated after transmission");
    
    $display("\nAll tests completed");
    #1000;
    $finish;
  end

  // Simplified monitor (removed internal signal access)
  always @(posedge clk) begin
    if ($time > 0 && rst_n) begin
      $display("T=%8tns: TX=%b, Busy=%b, RX_ready=%b, Error=%b",
        $time, 
        uo_out[0], 
        uo_out[1], 
        uo_out[2], 
        uo_out[3]);
    end
  end

endmodule
