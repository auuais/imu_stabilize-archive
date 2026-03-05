// I2C Slave with 16 x 8-bit registers
// Device address: 0x36 (7-bit)
// Register address: 0x00 ~ 0x0F
//
// Notes:
// - No external reset pin used. Internal POR reset generated from SCLK for POR_MS.
// - SDA is true open-drain: drive LOW only, otherwise Z (external pull-up required).
// - SDA changes only while SCL low (I2C spec), stable while SCL high.
// - START/STOP detection is qualified to avoid false STOP during SCL transitions.
// - dbg_last_wr_* mirrors added to confirm actual register write in ILA.

module Kintex_top_I2C_test #(
    parameter [6:0] SLAVE_ADDR = 7'h36,

    // ===== Internal POR reset params =====
    parameter integer SCLK_HZ = 100_000_000, // 100 MHz default
    parameter integer POR_MS  = 100,         // 100 ms reset

    // Register initial values
    parameter [7:0] INIT0  = 8'h00,
    parameter [7:0] INIT1  = 8'h01,
    parameter [7:0] INIT2  = 8'h02,
    parameter [7:0] INIT3  = 8'h03,
    parameter [7:0] INIT4  = 8'h04,
    parameter [7:0] INIT5  = 8'h05,
    parameter [7:0] INIT6  = 8'h06,
    parameter [7:0] INIT7  = 8'h07,
    parameter [7:0] INIT8  = 8'h08,
    parameter [7:0] INIT9  = 8'h09,
    parameter [7:0] INIT10 = 8'h0A,
    parameter [7:0] INIT11 = 8'h0B,
    parameter [7:0] INIT12 = 8'h0C,
    parameter [7:0] INIT13 = 8'h0D,
    parameter [7:0] INIT14 = 8'h0E,
    parameter [7:0] INIT15 = 8'h0F
)(
    input  wire FPGA_RESET, // unused
    input  wire SCLK_IN,
    input  wire SCL,
    inout  wire SDA,
	output wire [3:0] cam_select
);

    //===========================================================
    // SCLK BUFG
    //===========================================================
    wire SCLK;
    BUFG u_bufg_sclk (
        .I (SCLK_IN),
        .O (SCLK)
    );

    //===========================================================
    // Internal POR reset (POR_MS)
    //===========================================================
    function integer clog2;
        input integer value;
        integer i;
        begin
            clog2 = 0;
            for (i = value - 1; i > 0; i = i >> 1)
                clog2 = clog2 + 1;
        end
    endfunction

    localparam integer POR_CYCLES = (SCLK_HZ * POR_MS) / 1000;
    localparam integer POR_W      = (POR_CYCLES <= 1) ? 1 : clog2(POR_CYCLES + 1);

    reg  [POR_W-1:0] por_cnt /* synthesis preserve */;
    wire             nRESET_INT;

    assign nRESET_INT = (POR_CYCLES <= 1) ? 1'b1
                                          : (por_cnt >= POR_CYCLES[POR_W-1:0]);

    always @(posedge SCLK) begin
        if (!nRESET_INT)
            por_cnt <= por_cnt + {{(POR_W-1){1'b0}}, 1'b1};
        else
            por_cnt <= por_cnt;
    end

    //===========================================================
    // IBUF for SCL
    //===========================================================
    wire scl_ibuf;
    IBUF u_ibuf_scl (
        .I(SCL),
        .O(scl_ibuf)
    );

    //===========================================================
    // IOBUF for SDA (Open-Drain: low only)
    //===========================================================
    reg  sda_oe;     // 1 => drive LOW, 0 => release(Z)
    wire sda_in;

    wire sda_i;
    wire sda_o;
    wire sda_t;

    assign sda_o = 1'b0;
    assign sda_t = ~sda_oe;

    IOBUF u_iobuf_sda (
        .I (sda_o),
        .O (sda_i),
        .T (sda_t),
        .IO(SDA)
    );

    assign sda_in = sda_i;

    //===========================================================
    // Sync SCL and SDA to SCLK
    //===========================================================
    reg scl_meta, scl_sync, scl_sync_d;
    reg sda_meta, sda_sync, sda_sync_d;

    wire scl_rise =  scl_sync & ~scl_sync_d;
    wire scl_fall = ~scl_sync &  scl_sync_d;

    wire sda_rise =  sda_sync & ~sda_sync_d;
    wire sda_fall = ~sda_sync &  sda_sync_d;

    // ---- START/STOP qualification to avoid false detection ----
    wire scl_high_qual = scl_meta & scl_sync; // stable HIGH only

    wire start_cond = sda_fall & scl_high_qual; // START: SDA fall while SCL=1
    wire stop_cond  = sda_rise & scl_high_qual; // STOP : SDA rise while SCL=1

    //===========================================================
    // Register file (16 x 8-bit)
    //===========================================================
    reg [7:0] regfile [0:15];
    
    assign cam_select = regfile[0][3:0];
    reg [3:0] reg_index;

    //===========================================================
    // FSM / Shifters
    //===========================================================
    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_ADDR_RX   = 4'd1,
        ST_ADDR_ACK  = 4'd2,
        ST_REG_RX    = 4'd3,
        ST_REG_ACK   = 4'd4,
        ST_WRITE_RX  = 4'd5,
        ST_WRITE_ACK = 4'd6,
        ST_READ_TX   = 4'd7,
        ST_READ_ACK  = 4'd8;

    reg [3:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_reg;
    reg       rw_flag; // 0=write, 1=read
    reg       addr_match;

    //===========================================================
    // Debug signals (ILA)
    //===========================================================
     /*
     wire dbg_sclk    = SCLK;
     wire dbg_rstn    = nRESET_INT;
     wire dbg_scl     = scl_ibuf;
     wire dbg_sda     = sda_in;
     wire dbg_sclsync = scl_sync;
     wire dbg_sdasyn  = sda_sync;
     wire dbg_start   = start_cond;
     wire dbg_stop    = stop_cond;
     wire dbg_sclr    = scl_rise;
     wire dbg_sclf    = scl_fall;
     wire dbg_sdaoe   = sda_oe;

     reg  [3:0] dbg_state;
     reg  [2:0] dbg_bit_cnt;
     reg  [7:0] dbg_shift_reg;
     reg  [3:0] dbg_reg_index;
     reg        dbg_addr_match;
     reg        dbg_rw;

    // Write debug mirrors (ILA friendly)
     reg [3:0] dbg_last_wr_idx;
     reg [7:0] dbg_last_wr_data;
     reg       dbg_last_wr_pulse;

    always @(posedge SCLK) begin
        dbg_state      <= state;
        dbg_bit_cnt    <= bit_cnt;
        dbg_shift_reg  <= shift_reg;
        dbg_reg_index  <= reg_index;
        dbg_addr_match <= addr_match;
        dbg_rw         <= rw_flag;
    end
    */

    //===========================================================
    // Input synchronizers
    //===========================================================
    always @(posedge SCLK or negedge nRESET_INT) begin
        if (!nRESET_INT) begin
            scl_meta   <= 1'b0;
            scl_sync   <= 1'b0;
            scl_sync_d <= 1'b0;

            sda_meta   <= 1'b1;
            sda_sync   <= 1'b1;
            sda_sync_d <= 1'b1;
        end else begin
            scl_meta   <= scl_ibuf;
            scl_sync   <= scl_meta;
            scl_sync_d <= scl_sync;

            sda_meta   <= sda_in;
            sda_sync   <= sda_meta;
            sda_sync_d <= sda_sync;
        end
    end

    //===========================================================
    // Main FSM (RX on SCL rising, drive SDA only while SCL low)
    //===========================================================
    always @(posedge SCLK or negedge nRESET_INT) begin
        if (!nRESET_INT) begin
            state      <= ST_IDLE;
            bit_cnt    <= 3'd0;
            shift_reg  <= 8'd0;
            rw_flag    <= 1'b0;
            addr_match <= 1'b0;
            reg_index  <= 4'd0;

            regfile[0]  <= INIT0;
            regfile[1]  <= INIT1;
            regfile[2]  <= INIT2;
            regfile[3]  <= INIT3;
            regfile[4]  <= INIT4;
            regfile[5]  <= INIT5;
            regfile[6]  <= INIT6;
            regfile[7]  <= INIT7;
            regfile[8]  <= INIT8;
            regfile[9]  <= INIT9;
            regfile[10] <= INIT10;
            regfile[11] <= INIT11;
            regfile[12] <= INIT12;
            regfile[13] <= INIT13;
            regfile[14] <= INIT14;
            regfile[15] <= INIT15;

            sda_oe <= 1'b0;

            //dbg_last_wr_idx   <= 4'd0;
            //dbg_last_wr_data  <= 8'd0;
            //dbg_last_wr_pulse <= 1'b0;

        end else begin
            // default pulse low (one-shot)
            //dbg_last_wr_pulse <= 1'b0;

            // STOP: release bus, go idle
            if (stop_cond) begin
                state      <= ST_IDLE;
                sda_oe     <= 1'b0;
                addr_match <= 1'b0;
            end

            // START or Re-START: go receive address
            if (start_cond) begin
                state      <= ST_ADDR_RX;
                bit_cnt    <= 3'd7;
                shift_reg  <= 8'd0;
                sda_oe     <= 1'b0;
                addr_match <= 1'b0;
            end

            // DRIVE SDA only while SCL LOW (ACK / READ data)
            if (!scl_sync) begin
                case (state)
                    ST_ADDR_ACK:  sda_oe <= addr_match ? 1'b1 : 1'b0;
                    ST_REG_ACK:   sda_oe <= addr_match ? 1'b1 : 1'b0;
                    ST_WRITE_ACK: sda_oe <= addr_match ? 1'b1 : 1'b0;

                    ST_READ_TX: begin
                        if (!addr_match)
                            sda_oe <= 1'b0;
                        else
                            // open-drain: 0 drives low, 1 releases
                            sda_oe <= (regfile[reg_index][bit_cnt] == 1'b0) ? 1'b1 : 1'b0;
                    end

                    ST_READ_ACK:  sda_oe <= 1'b0; // master drives ACK/NACK

                    default:      sda_oe <= 1'b0; // RX states: release
                endcase
            end

            // SAMPLE on SCL rising edge
            if (scl_rise) begin
                case (state)

                    ST_ADDR_RX: begin
                        shift_reg[bit_cnt] <= sda_sync;
                        if (bit_cnt == 0) begin
                            // address bits already in shift_reg[7:1]
                            rw_flag    <= sda_sync;                 // R/W bit
                            addr_match <= (shift_reg[7:1] == SLAVE_ADDR); // Verilog-safe
                            state      <= ST_ADDR_ACK;
                            bit_cnt    <= 3'd7;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end

                    ST_ADDR_ACK: begin
                        if (!addr_match) begin
                            state <= ST_IDLE;
                        end else if (rw_flag == 1'b0) begin
                            state     <= ST_REG_RX;
                            bit_cnt   <= 3'd7;
                            shift_reg <= 8'd0;
                        end else begin
                            state   <= ST_READ_TX;
                            bit_cnt <= 3'd7;
                        end
                    end

                    ST_REG_RX: begin
                        shift_reg[bit_cnt] <= sda_sync;
                        if (bit_cnt == 0) begin
                            reg_index <= ({shift_reg[7:1], sda_sync} & 8'h0F);
                            state     <= ST_REG_ACK;
                            bit_cnt   <= 3'd7;
                            shift_reg <= 8'd0;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end

                    ST_REG_ACK: begin
                        state     <= ST_WRITE_RX;
                        bit_cnt   <= 3'd7;
                        shift_reg <= 8'd0;
                    end

                    ST_WRITE_RX: begin
                        shift_reg[bit_cnt] <= sda_sync;
                        if (bit_cnt == 0) begin
                            // WRITE the received data byte
                            regfile[reg_index] <= {shift_reg[7:1], sda_sync};

                            // Debug mirrors (confirm actual write happened)
                            //dbg_last_wr_idx   <= reg_index;
                            //dbg_last_wr_data  <= {shift_reg[7:1], sda_sync};
                            //dbg_last_wr_pulse <= 1'b1;

                            reg_index <= reg_index + 1'b1;
                            state     <= ST_WRITE_ACK;
                            bit_cnt   <= 3'd7;
                            shift_reg <= 8'd0;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end

                    ST_WRITE_ACK: begin
                        state     <= ST_WRITE_RX;
                        bit_cnt   <= 3'd7;
                        shift_reg <= 8'd0;
                    end

                    ST_READ_TX: begin
                        if (!addr_match) begin
                            state <= ST_IDLE;
                        end else if (bit_cnt == 0) begin
                            state <= ST_READ_ACK;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end

                    ST_READ_ACK: begin
                        if (sda_sync == 1'b0) begin
                            reg_index <= reg_index + 1'b1;
                            state     <= ST_READ_TX;
                            bit_cnt   <= 3'd7;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end

                    default: begin
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end
 

endmodule
