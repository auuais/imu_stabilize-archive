// ============================================================================
//
// cam_select encoding (regfile[0][2:0])
//   0: CAM0
//   1: CAM1
//   2: CAM2
//   3: CAM3
//   4: CAM4
//   5: CAM5
//   others: CAM5 (default)
// ============================================================================

module KintexTop_Cam_6ch_SDI(
    // FPGA global reset
    input  wire        FPGA_RESET,

    // I2C slave (camera select)
    input  wire        SCL,
    inout  wire        SDA,

    // Strobe input (to camera TRIG)
    input  wire        STROBE_OUT0,

    // EO Camera 0
    input  wire        CAM0_PCLK,
    input  wire [7:0]  CAM0_YOUT,
    input  wire [7:0]  CAM0_COUT,

    // EO Camera 1
    input  wire        CAM1_PCLK,
    input  wire [7:0]  CAM1_YOUT,
    input  wire [7:0]  CAM1_COUT,
    output wire        TRIG_IN1,

    // EO Camera 2
    input  wire        CAM2_PCLK,
    input  wire [7:0]  CAM2_YOUT,
    input  wire [7:0]  CAM2_COUT,
    output wire        TRIG_IN2,

    // EO Camera 3
    input  wire        CAM3_PCLK,
    input  wire [7:0]  CAM3_YOUT,
    input  wire [7:0]  CAM3_COUT,
    output wire        TRIG_IN3,

    // EO Camera 4
    input  wire        CAM4_PCLK,
    input  wire [7:0]  CAM4_YOUT,
    input  wire [7:0]  CAM4_COUT,
    output wire        TRIG_IN4,

    // EO Camera 5
    input  wire        CAM5_PCLK,
    input  wire [7:0]  CAM5_YOUT,
    input  wire [7:0]  CAM5_COUT,
    output wire        TRIG_IN5,

    // HD-SDI 
	//////////////////////////////
    output wire         HD_DE,
    output wire         HD_VSYNC,
    output wire         HD_HSYNC,
    output wire         HD_PCLK,
    output wire [19:0]  HD_DOUT,
	//////////////////////////////

    output wire        IEG0_PCLK,
    output wire        IEG0_HSYNC,
    output wire        IEG0_VSYNC,
    output wire [19:0] IEG0_DOUT

    //output wire        IEG1_PCLK,
    //output wire        IEG1_HSYNC,
    //output wire        IEG1_VSYNC,
    //output wire [19:0] IEG1_DOUT
);

/////////////////////////////////////////////
// Reset buffer
/////////////////////////////////////////////
wire nRESET;
assign	nRESET = 1'b1;
/*
wire nRESET
IBUF U_RESET_bufg (
   .I(FPGA_RESET),
   .O(nRESET)
);
*/



// ------------------------------------------------------------------------
// PCLK buffering for HDIO banks (fix DRC PLHDIO-3)
// - CAM0~CAM3_PCLK are in HIGH_DENSITY IO banks; buffer them with BUFGCE
//   before feeding any BUFGCTRL/clock-mux logic.
//wire CAM0_PCLK_BUFG;
//wire CAM1_PCLK_BUFG;
//wire CAM2_PCLK_BUFG;
//wire CAM3_PCLK_BUFG;
//wire CAM4_PCLK_BUFG;
//wire CAM5_PCLK_BUFG;

IBUF u_ibuf_cam0_pclk (.I(CAM0_PCLK), .O(CAM0_PCLK_BUFG));
//BUFGCE u_bufgce_cam1_pclk (.I(CAM1_PCLK), .CE(1'b1), .O(CAM1_PCLK_BUFG));
//BUFGCE u_bufgce_cam2_pclk (.I(CAM2_PCLK), .CE(1'b1), .O(CAM2_PCLK_BUFG));
//BUFGCE u_bufgce_cam3_pclk (.I(CAM3_PCLK), .CE(1'b1), .O(CAM3_PCLK_BUFG));
//BUFGCE u_bufgce_cam4_pclk (.I(CAM4_PCLK), .CE(1'b1), .O(CAM4_PCLK_BUFG));
//BUFGCE u_bufgce_cam5_pclk (.I(CAM5_PCLK), .CE(1'b1), .O(CAM5_PCLK_BUFG));

 // ------------------------------------------------------------------------
 // I2C slave instance (register file)
 // ------------------------------------------------------------------------
 //wire [2:0]	cam_select;
 wire [3:0]	cam_select;
 	Kintex_top_I2C_test u_i2c (
     .FPGA_RESET (nRESET),
     .SCLK_IN    (CAM0_PCLK_BUFG),
     .SCL        (SCL),
     .SDA        (SDA),
     .cam_select (cam_select)
 );
 
 
 // ------------------------------------------------------------------------
 // Instantiate each single-cam top (0~5)
 // ------------------------------------------------------------------------
 // CAM0 (0cam top은 IEG0_DOUT 폭이 [19:2]라서 여기서 [19:0]로 padding)
 wire        cam0_IEG0_PCLK;
 wire        cam0_IEG0_HSYNC;
 wire        cam0_IEG0_VSYNC;
 wire [19:2] cam0_IEG0_DOUT_19_2;
 wire [19:0] cam0_IEG0_DOUT = {cam0_IEG0_DOUT_19_2, 2'b00};
 wire        cam0_IEG0_TEST0;
 wire        cam0_IEG0_TEST1;
 
 wire        cam0_IEG1_PCLK;
 wire        cam0_IEG1_HSYNC;
 wire        cam0_IEG1_VSYNC;
 wire [19:0] cam0_IEG1_DOUT;



    Kintex_top_0cam_1ch u_cam0 (
        .FPGA_RESET (nRESET),

        .CAM0_PCLK  (CAM0_PCLK_BUFG),
        //.CAM0_PCLK  (CAM0_PCLK),
        .CAM0_YOUT  (CAM0_YOUT),
        .CAM0_COUT  (CAM0_COUT),

		//.HD_DE		(		),
		//.HD_VSYNC	(		),
		//.HD_HSYNC	(		),
		//.HD_PCLK	(		),
		//.HD_DOUT	(		),

        .IEG0_PCLK  (cam0_IEG0_PCLK),
        .IEG0_HSYNC (cam0_IEG0_HSYNC),
        .IEG0_VSYNC (cam0_IEG0_VSYNC),
        .IEG0_DOUT  (cam0_IEG0_DOUT_19_2),

        .IEG1_PCLK  (cam0_IEG1_PCLK),
        .IEG1_HSYNC (cam0_IEG1_HSYNC),
        .IEG1_VSYNC (cam0_IEG1_VSYNC),
        .IEG1_DOUT  (cam0_IEG1_DOUT)
    );

    // CAM1
    wire        cam1_IEG0_PCLK;
    wire        cam1_IEG0_HSYNC;
    wire        cam1_IEG0_VSYNC;
    wire [19:2] cam1_IEG0_DOUT_19_2;
    wire [19:0] cam1_IEG0_DOUT = {cam1_IEG0_DOUT_19_2, 2'b00};

    wire        cam1_IEG1_PCLK;
    wire        cam1_IEG1_HSYNC;
    wire        cam1_IEG1_VSYNC;
    wire [19:0] cam1_IEG1_DOUT;

    Kintex_top_1cam_1ch u_cam1 (
        .FPGA_RESET (nRESET),
        //.CAM1_PCLK  (CAM1_PCLK_BUFG),
        .CAM1_PCLK  (CAM1_PCLK),
        .CAM1_YOUT  (CAM1_YOUT),
        .CAM1_COUT  (CAM1_COUT),
        .STROBE_OUT0(STROBE_OUT0),
        .TRIG_IN1   (TRIG_IN1),
        .IEG0_PCLK  (cam1_IEG0_PCLK),
        .IEG0_HSYNC (cam1_IEG0_HSYNC),
        .IEG0_VSYNC (cam1_IEG0_VSYNC),
        .IEG0_DOUT  (cam1_IEG0_DOUT),
        .IEG1_PCLK  (cam1_IEG1_PCLK),
        .IEG1_HSYNC (cam1_IEG1_HSYNC),
        .IEG1_VSYNC (cam1_IEG1_VSYNC),
        .IEG1_DOUT  (cam1_IEG1_DOUT)
    );

    // CAM2
    wire        cam2_IEG0_PCLK;
    wire        cam2_IEG0_HSYNC;
    wire        cam2_IEG0_VSYNC;
    wire [19:2] cam2_IEG0_DOUT_19_2;
    wire [19:0] cam2_IEG0_DOUT = {cam2_IEG0_DOUT_19_2, 2'b00};

    wire        cam2_IEG1_PCLK;
    wire        cam2_IEG1_HSYNC;
    wire        cam2_IEG1_VSYNC;
    wire [19:0] cam2_IEG1_DOUT;

    Kintex_top_2cam_1ch u_cam2 (
        .FPGA_RESET (nRESET),
        //.CAM2_PCLK  (CAM2_PCLK_BUFG),
        .CAM2_PCLK  (CAM2_PCLK),
        .CAM2_YOUT  (CAM2_YOUT),
        .CAM2_COUT  (CAM2_COUT),
        .STROBE_OUT0(STROBE_OUT0),
        .TRIG_IN2   (TRIG_IN2),
        .IEG0_PCLK  (cam2_IEG0_PCLK),
        .IEG0_HSYNC (cam2_IEG0_HSYNC),
        .IEG0_VSYNC (cam2_IEG0_VSYNC),
        .IEG0_DOUT  (cam2_IEG0_DOUT),
        .IEG1_PCLK  (cam2_IEG1_PCLK),
        .IEG1_HSYNC (cam2_IEG1_HSYNC),
        .IEG1_VSYNC (cam2_IEG1_VSYNC),
        .IEG1_DOUT  (cam2_IEG1_DOUT)
    );

    // CAM3
    wire        cam3_IEG0_PCLK;
    wire        cam3_IEG0_HSYNC;
    wire        cam3_IEG0_VSYNC;
    wire [19:2] cam3_IEG0_DOUT_19_2;
    wire [19:0] cam3_IEG0_DOUT = {cam3_IEG0_DOUT_19_2, 2'b00};

    wire        cam3_IEG1_PCLK;
    wire        cam3_IEG1_HSYNC;
    wire        cam3_IEG1_VSYNC;
    wire [19:0] cam3_IEG1_DOUT;

    Kintex_top_3cam_1ch u_cam3 (
        .FPGA_RESET (nRESET),
        //.CAM3_PCLK  (CAM3_PCLK_BUFG),
        .CAM3_PCLK  (CAM3_PCLK),
        .CAM3_YOUT  (CAM3_YOUT),
        .CAM3_COUT  (CAM3_COUT),
        .STROBE_OUT0(STROBE_OUT0),
        .TRIG_IN3   (TRIG_IN3),
        .IEG0_PCLK  (cam3_IEG0_PCLK),
        .IEG0_HSYNC (cam3_IEG0_HSYNC),
        .IEG0_VSYNC (cam3_IEG0_VSYNC),
        .IEG0_DOUT  (cam3_IEG0_DOUT),
        .IEG1_PCLK  (cam3_IEG1_PCLK),
        .IEG1_HSYNC (cam3_IEG1_HSYNC),
        .IEG1_VSYNC (cam3_IEG1_VSYNC),
        .IEG1_DOUT  (cam3_IEG1_DOUT)
    );

    // CAM4
    wire        cam4_IEG0_PCLK;
    wire        cam4_IEG0_HSYNC;
    wire        cam4_IEG0_VSYNC;
    wire [19:2] cam4_IEG0_DOUT_19_2;
    wire [19:0] cam4_IEG0_DOUT = {cam4_IEG0_DOUT_19_2, 2'b00};

    wire        cam4_IEG1_PCLK;
    wire        cam4_IEG1_HSYNC;
    wire        cam4_IEG1_VSYNC;
    wire [19:0] cam4_IEG1_DOUT;

    Kintex_top_4cam_1ch u_cam4 (
        .FPGA_RESET (nRESET),
        //.CAM4_PCLK  (CAM4_PCLK_BUFG),
        .CAM4_PCLK  (CAM4_PCLK),
        .CAM4_YOUT  (CAM4_YOUT),
        .CAM4_COUT  (CAM4_COUT),
        .STROBE_OUT0(STROBE_OUT0),
        .TRIG_IN4   (TRIG_IN4),
        .IEG0_PCLK  (cam4_IEG0_PCLK),
        .IEG0_HSYNC (cam4_IEG0_HSYNC),
        .IEG0_VSYNC (cam4_IEG0_VSYNC),
        .IEG0_DOUT  (cam4_IEG0_DOUT),
        .IEG1_PCLK  (cam4_IEG1_PCLK),
        .IEG1_HSYNC (cam4_IEG1_HSYNC),
        .IEG1_VSYNC (cam4_IEG1_VSYNC),
        .IEG1_DOUT  (cam4_IEG1_DOUT)
    );

    // CAM5
    wire        cam5_IEG0_PCLK;

/*
// Selected write-side clock for the IEG0 FIFO (tracks the selected camera clock domain)
wire IEG0_WR_PCLK = (cam_select == 4'd0) ? cam0_IEG0_PCLK :
                   (cam_select == 4'd1) ? cam1_IEG0_PCLK :
                   (cam_select == 4'd2) ? cam2_IEG0_PCLK :
                   (cam_select == 4'd3) ? cam3_IEG0_PCLK :
                   (cam_select == 4'd4) ? cam4_IEG0_PCLK :
                                          cam5_IEG0_PCLK;
*/

// 1. MUX 출력을 임시 wire로 받음
wire IEG0_WR_PCLK_MUX_OUT = (cam_select == 4'd0) ? cam0_IEG0_PCLK :
                            (cam_select == 4'd1) ? cam1_IEG0_PCLK :
                            (cam_select == 4'd2) ? cam2_IEG0_PCLK :
                            (cam_select == 4'd3) ? cam3_IEG0_PCLK :
                            (cam_select == 4'd4) ? cam4_IEG0_PCLK :
                                                   cam5_IEG0_PCLK;

wire IEG0_WR_PCLK_BUFG;

BUFG u_clk_mux_bufg (
    .I (IEG0_WR_PCLK_MUX_OUT),
    .O (IEG0_WR_PCLK_BUFG)
);

    wire        cam5_IEG0_HSYNC;
    wire        cam5_IEG0_VSYNC;
    wire [19:2] cam5_IEG0_DOUT_19_2;
    wire [19:0] cam5_IEG0_DOUT = {cam5_IEG0_DOUT_19_2, 2'b00};

    wire        cam5_IEG1_PCLK;
    wire        cam5_IEG1_HSYNC;
    wire        cam5_IEG1_VSYNC;
    wire [19:0] cam5_IEG1_DOUT;

    Kintex_top_5cam_1ch u_cam5 (
        .FPGA_RESET (nRESET),
        .CAM5_PCLK  (CAM5_PCLK),
        .CAM5_YOUT  (CAM5_YOUT),
        .CAM5_COUT  (CAM5_COUT),
        .STROBE_OUT0(STROBE_OUT0),
        .TRIG_IN5   (TRIG_IN5),
        .IEG0_PCLK  (cam5_IEG0_PCLK),
        .IEG0_HSYNC (cam5_IEG0_HSYNC),
        .IEG0_VSYNC (cam5_IEG0_VSYNC),
        .IEG0_DOUT  (cam5_IEG0_DOUT),
        .IEG1_PCLK  (cam5_IEG1_PCLK),
        .IEG1_HSYNC (cam5_IEG1_HSYNC),
        .IEG1_VSYNC (cam5_IEG1_VSYNC),
        .IEG1_DOUT  (cam5_IEG1_DOUT)
    );

wire [19:0] IEG0_DOUT_MUX;
wire [19:0] IEG0_DOUT_FIFO_DOUT;
wire        IEG0_DOUT_FIFO_EMPTY;
wire        IEG0_DOUT_FIFO_FULL;
wire        IEG0_DOUT_FIFO_RDEN;
wire IEG0_DOUT_FIFO_WR_RST_BUSY;
wire IEG0_DOUT_FIFO_RD_RST_BUSY;

 // ------------------------------------------------------------------------
 // MUX: Select IEG outputs by cam_select
 // ------------------------------------------------------------------------
 
 
 assign IEG0_HSYNC = (cam_select == 4'd0) ? cam0_IEG0_HSYNC :
                     (cam_select == 4'd1) ? cam1_IEG0_HSYNC :
                     (cam_select == 4'd2) ? cam2_IEG0_HSYNC :
                     (cam_select == 4'd3) ? cam3_IEG0_HSYNC :
                     (cam_select == 4'd4) ? cam4_IEG0_HSYNC :
                     (cam_select == 4'd5) ? cam5_IEG0_HSYNC :
                                            cam5_IEG0_HSYNC;
 
 assign IEG0_VSYNC = (cam_select == 4'd0) ? cam0_IEG0_VSYNC :
                     (cam_select == 4'd1) ? cam1_IEG0_VSYNC :
                     (cam_select == 4'd2) ? cam2_IEG0_VSYNC :
                     (cam_select == 4'd3) ? cam3_IEG0_VSYNC :
                     (cam_select == 4'd4) ? cam4_IEG0_VSYNC :
                     (cam_select == 4'd5) ? cam5_IEG0_VSYNC :
                                            cam5_IEG0_VSYNC;
 
 assign IEG0_DOUT_MUX = (cam_select == 4'd0) ? cam0_IEG0_DOUT  :
                        (cam_select == 4'd1) ? cam1_IEG0_DOUT  :
                        (cam_select == 4'd2) ? cam2_IEG0_DOUT  :
                        (cam_select == 4'd3) ? cam3_IEG0_DOUT  :
                        (cam_select == 4'd4) ? cam4_IEG0_DOUT  :
                        (cam_select == 4'd5) ? cam5_IEG0_DOUT  :
                                            cam5_IEG0_DOUT;

/* 
    assign IEG1_HSYNC = (cam_select == 4'd0) ? cam0_IEG1_HSYNC :
                        (cam_select == 4'd1) ? cam1_IEG1_HSYNC :
                        (cam_select == 4'd2) ? cam2_IEG1_HSYNC :
                        (cam_select == 4'd3) ? cam3_IEG1_HSYNC :
                        (cam_select == 4'd4) ? cam4_IEG1_HSYNC :
                        (cam_select == 4'd5) ? cam5_IEG1_HSYNC :
                                               cam5_IEG1_HSYNC;

    assign IEG1_VSYNC = (cam_select == 4'd0) ? cam0_IEG1_VSYNC :
                        (cam_select == 4'd1) ? cam1_IEG1_VSYNC :
                        (cam_select == 4'd2) ? cam2_IEG1_VSYNC :
                        (cam_select == 4'd3) ? cam3_IEG1_VSYNC :
                        (cam_select == 4'd4) ? cam4_IEG1_VSYNC :
                        (cam_select == 4'd5) ? cam5_IEG1_VSYNC :
                                               cam5_IEG1_VSYNC;

    assign IEG1_DOUT  = (cam_select == 4'd0) ? cam0_IEG1_DOUT  :
                        (cam_select == 4'd1) ? cam1_IEG1_DOUT  :
                        (cam_select == 4'd2) ? cam2_IEG1_DOUT  :
                        (cam_select == 4'd3) ? cam3_IEG1_DOUT  :
                        (cam_select == 4'd4) ? cam4_IEG1_DOUT  :
                        (cam_select == 4'd5) ? cam5_IEG1_DOUT  :
                                               cam5_IEG1_DOUT;
*/
									   

//assign IEG1_PCLK  = IEG0_WR_PCLK_BUFG  ;

//assign IEG1_PCLK  = 1'b0;
//assign IEG1_HSYNC = 1'b0;
//assign IEG1_VSYNC = 1'b0;
//assign IEG1_DOUT  = 20'b0;


assign IEG0_DOUT_FIFO_RDEN = ~IEG0_DOUT_FIFO_EMPTY;

// FIFO 출력이 없으면 0 출력
assign IEG0_DOUT = (IEG0_DOUT_FIFO_EMPTY) ? 20'h0 : IEG0_DOUT_FIFO_DOUT;
assign IEG0_PCLK  = IEG0_WR_PCLK_BUFG  ;

// ============================================================
reg [3:0] cam_select_d;
reg [7:0] reset_counter;
reg       auto_reset_active;

always @(posedge IEG0_WR_PCLK_BUFG) begin
    cam_select_d <= cam_select;
    
    if (cam_select != cam_select_d) begin
        reset_counter <= 8'hFF; 
        auto_reset_active <= 1'b1;
    end
    else if (reset_counter > 0) begin
        reset_counter <= reset_counter - 1;
        auto_reset_active <= 1'b1;
    end
    else begin
        auto_reset_active <= 1'b0;
    end
end
// ============================================================

// Xilinx XPM synchronous FIFO (single-clock)
// IEG0 DOUT FIFO: async FIFO to safely cross from selected camera PCLK to IEG0_PCLK
xpm_fifo_async #(
    .DOUT_RESET_VALUE        ("0"),
    .ECC_MODE                ("no_ecc"),
    .FIFO_MEMORY_TYPE        ("auto"),
    .FIFO_READ_LATENCY       (0),
    .FIFO_WRITE_DEPTH        (1024),
    .FULL_RESET_VALUE        (0),
    .PROG_EMPTY_THRESH       (10),
    .PROG_FULL_THRESH        (1014),
    .RD_DATA_COUNT_WIDTH     (10),
    .READ_DATA_WIDTH         (20),
    .READ_MODE               ("fwft"),
    .SIM_ASSERT_CHK          (0),
    .USE_ADV_FEATURES        ("0000"),
    .WAKEUP_TIME             (0),
    .WR_DATA_COUNT_WIDTH     (10),
    .WRITE_DATA_WIDTH        (20),
    .CDC_SYNC_STAGES         (2),
    .RELATED_CLOCKS          (0)





) u_fifo_ieg0_dout (
    //.rst            (~nRESET),
    .rst            (auto_reset_active),

    //.wr_clk         (IEG0_WR_PCLK),
 	.wr_clk (IEG0_WR_PCLK_BUFG), 
    .rd_clk         (IEG0_PCLK),

    .din            (IEG0_DOUT_MUX),
    .wr_en          (IEG0_HSYNC & ~IEG0_DOUT_FIFO_WR_RST_BUSY),
    .full           (IEG0_DOUT_FIFO_FULL),

    .rd_en          (IEG0_DOUT_FIFO_RDEN & ~IEG0_DOUT_FIFO_RD_RST_BUSY),
    .dout           (IEG0_DOUT_FIFO_DOUT),
    .empty          (IEG0_DOUT_FIFO_EMPTY),

    .wr_rst_busy    (IEG0_DOUT_FIFO_WR_RST_BUSY),
    .rd_rst_busy    (IEG0_DOUT_FIFO_RD_RST_BUSY),

    .sleep          (1'b0),
    .injectsbiterr  (1'b0),
    .injectdbiterr  (1'b0)
);



/*
assign HD_PCLK  = IEG0_WR_PCLK_BUFG  ;
//assign HD_DE    = cam0_hsync1920; 
//assign HD_VSYNC = cam0_vsync;
//assign HD_HSYNC = cam0_hsync1920 & !cam0_vsync;

assign HD_DE = 1'b0;
assign HD_HSYNC = (cam_select == 4'd0) ? cam0_IEG0_HSYNC :
                  (cam_select == 4'd1) ? cam1_IEG0_HSYNC :
                  (cam_select == 4'd2) ? cam2_IEG0_HSYNC :
                  (cam_select == 4'd3) ? cam3_IEG0_HSYNC :
                  (cam_select == 4'd4) ? cam4_IEG0_HSYNC :
                  (cam_select == 4'd5) ? cam5_IEG0_HSYNC : cam5_IEG0_HSYNC;
assign HD_VSYNC = (cam_select == 4'd0) ? cam0_IEG0_VSYNC :
                  (cam_select == 4'd1) ? cam1_IEG0_VSYNC :
                  (cam_select == 4'd2) ? cam2_IEG0_VSYNC :
                  (cam_select == 4'd3) ? cam3_IEG0_VSYNC : 
                  (cam_select == 4'd4) ? cam4_IEG0_VSYNC :
                  (cam_select == 4'd5) ? cam5_IEG0_VSYNC : cam5_IEG0_VSYNC;

assign HD_DOUT  = (IEG0_DOUT_FIFO_EMPTY) ? 20'h0 : IEG0_DOUT_FIFO_DOUT;
*/
assign HD_PCLK  = IEG0_WR_PCLK_BUFG  ;
assign HD_DE    = 1'b1; 
assign HD_VSYNC = 1'b1;
assign HD_HSYNC = 1'b1;
assign HD_DOUT  = IEG0_DOUT_MUX; 


endmodule
