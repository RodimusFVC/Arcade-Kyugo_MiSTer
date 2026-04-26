//============================================================================
//
//  Kyugo CPU board
//  Copyright (C) 2026 Rodimus
//
//  MAME reference: kyugo.cpp
//  Hardware: Z80 CPU1 + Z80 CPU2 @ 3.072 MHz (XTAL 18.432 / 6)
//            2x AY-3-8910 @ 1.536 MHz (18.432 / 12)
//  Screen: 396x260 total, visible 288x224 (lines 16-239), 59.66 Hz
//
//============================================================================

module Kyugo_CPU
(
	input         reset,
	input         clk_49m,
	output  [4:0] red, green, blue,
	output        video_hsync, video_vsync, video_csync,
	output        video_hblank, video_vblank,
	output        ce_pix,
	input   [7:0] p1_controls,
	input   [7:0] p2_controls,
	input   [7:0] sys_controls,
	input  [15:0] dip_sw,
    input         rot_flip,
	output signed [15:0] sound,
	input   [3:0] h_center, v_center,
	input         main_rom_cs_i, sub_rom_cs_i, fg_rom_cs_i,
	input         bg0_rom_cs_i, bg1_rom_cs_i, bg2_rom_cs_i,
	input         spr0_rom_cs_i, spr1_rom_cs_i, spr2_rom_cs_i,
	input         prom_r_cs_i, prom_g_cs_i, prom_b_cs_i,
	input         prom_lut_cs_i, prom_tim_cs_i,
	input  [24:0] ioctl_addr,
	input   [7:0] ioctl_data,
	input         ioctl_wr,
	input   [2:0] variant_sel,
	output  [1:0] coin_counter,
	input         pause,
	input  [15:0] hs_address,
	input   [7:0] hs_data_in,
	output  [7:0] hs_data_out,
	input         hs_write
);

//------------------------------------------------------- Clock enables -------------------------------------------------------//

// Pixel clock: 49.152 MHz * 4/32 = 6.144 MHz = XTAL/3 (matches MAME set_raw(XTAL/3, 396, 0, 288, 260, 16, 240) → 59.6575 Hz)
wire [1:0] pix_cen_o;
jtframe_frac_cen #(2) pix_cen (.clk(clk_49m), .n(10'd4), .m(10'd32), .cen(pix_cen_o), .cenb());
wire cen_pix = pix_cen_o[0];
assign ce_pix = cen_pix;

reg [3:0] cpu_div = 4'd0;
always_ff @(posedge clk_49m) cpu_div <= cpu_div + 4'd1;
wire cen_cpu = (cpu_div == 4'd0);

reg ay_toggle = 1'b0;
always_ff @(posedge clk_49m) if (cen_cpu) ay_toggle <= ~ay_toggle;
wire cen_ay = cen_cpu & ~ay_toggle & ~pause;

//-------------------------------------------------------- Video timing --------------------------------------------------------//

reg [8:0] base_h_cnt = 9'd0;
reg [8:0] v_cnt      = 9'd0;

wire [8:0] h_cnt_rot;
wire [8:0] v_cnt_rot;

assign h_cnt_rot = { base_h_cnt[8], base_h_cnt[7:0] ^ {8{flip_screen}} };
assign v_cnt_rot = { v_cnt[8],      v_cnt[7:0]      ^ {8{flip_screen}} };

always_ff @(posedge clk_49m) begin
	if (cen_pix) begin
		if (base_h_cnt == 9'd395) begin
			base_h_cnt <= 9'd0;
			v_cnt <= (v_cnt == 9'd259) ? 9'd0 : v_cnt + 9'd1;
		end else begin
			base_h_cnt <= base_h_cnt + 9'd1;
		end
	end
end

wire hblk = (base_h_cnt >= 9'd288);
wire vblk = (v_cnt < 9'd16) | (v_cnt >= 9'd240);
assign video_hblank = hblk;
assign video_vblank = vblk;

wire [8:0] hs_start = 9'd300 + {5'd0, h_center};
wire [8:0] hs_end   = hs_start + 9'd16;
wire [8:0] vs_start = 9'd244 + {5'd0, v_center};
wire [8:0] vs_end   = vs_start + 9'd4;
assign video_hsync = (h_cnt_rot >= hs_start && h_cnt_rot < hs_end);
assign video_vsync = (v_cnt_rot >= vs_start && v_cnt_rot < vs_end);
assign video_csync = ~(video_hsync ^ video_vsync);

//------------------------------------------------------- CPU1 — Main ---------------------------------------------------------//

wire [15:0] cpu1_A;
wire [7:0]  cpu1_Dout;
wire        cpu1_WR_n, cpu1_RD_n, cpu1_MREQ_n, cpu1_IORQ_n, cpu1_M1_n, cpu1_RFSH_n;

T80s cpu1
(
	.RESET_n(reset), .CLK(clk_49m), .CEN(cen_cpu & ~pause), .WAIT_n(1'b1),
	.INT_n(1'b1), .NMI_n(~cpu1_nmi),
	.M1_n(cpu1_M1_n), .MREQ_n(cpu1_MREQ_n), .IORQ_n(cpu1_IORQ_n),
	.RD_n(cpu1_RD_n), .WR_n(cpu1_WR_n), .RFSH_n(cpu1_RFSH_n),
	.A(cpu1_A), .DI(cpu1_Din), .DO(cpu1_Dout)
);

// NMI: scanline 240, gated by nmi_mask, pulse (cleared when CPU acknowledges)
reg cpu1_nmi = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset) cpu1_nmi <= 0;
	else begin
		if (cen_pix && (base_h_cnt == 9'd0) && (v_cnt == 9'd240) && nmi_mask)
			cpu1_nmi <= 1;
		if (~cpu1_MREQ_n & ~cpu1_M1_n) cpu1_nmi <= 0;
	end
end

//-------------------------------------------------- CPU1 Address Decoding (variant-aware) ------------------------------//

localparam [2:0] VAR_GYRO     = 3'd0,
                 VAR_REPULSE  = 3'd1,
                 VAR_FLASHGAL = 3'd2,
                 VAR_SRDMISSN = 3'd3,
                 VAR_LEGEND   = 3'd4;

wire cpu1_mem_valid = ~cpu1_MREQ_n & cpu1_RFSH_n;
wire cs_rom       = cpu1_mem_valid & ~cpu1_A[15];                              // 0000-7FFF
wire cs_bgvram    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b10000);              // 8000-87FF
wire cs_bgattr    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b10001);              // 8800-8FFF
wire cs_fgvram    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b10010);              // 9000-97FF
wire cs_spram1    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b10011);              // 9800-9FFF
wire cs_spram0    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b10100);              // A000-A7FF
wire cs_scrollxlo = cpu1_mem_valid & ~cpu1_WR_n & (cpu1_A[15:0] == 16'hA800); // A800 W
wire cs_gfxctrl   = cpu1_mem_valid & ~cpu1_WR_n & (cpu1_A[15:0] == 16'hB000); // B000 W
wire cs_scrolly   = cpu1_mem_valid & ~cpu1_WR_n & (cpu1_A[15:0] == 16'hB800); // B800 W
// E000 region: Gyrodine = watchdog write, SRDMission = shared RAM mirror, others = unmapped
wire cs_e000_blk  = cpu1_mem_valid & (cpu1_A[15:11] == 5'b11100);              // E000-E7FF
wire cs_watchdog  = cs_e000_blk & (variant_sel == VAR_GYRO);
wire cs_shared_e  = cs_e000_blk & (variant_sel == VAR_SRDMISSN);
wire cs_shared    = cpu1_mem_valid & (cpu1_A[15:11] == 5'b11110);              // F000-F7FF
wire cs_mainlatch = ~cpu1_IORQ_n & ~cpu1_WR_n;                                 // IO 00-07 (global_mask 0x07)

// LS259 mainlatch (I/O port mapped, global_mask 0x07 → addr = cpu1_A[2:0])
reg [7:0] mainlatch = 8'd0;
always_ff @(posedge clk_49m) begin
	if (!reset) mainlatch <= 8'd0;
	else if (cen_cpu && cs_mainlatch) mainlatch[cpu1_A[2:0]] <= cpu1_Dout[0];
end
wire nmi_mask    = mainlatch[0];
wire flip_screen = rot_flip ^ mainlatch[1];
wire cpu2_rst    = ~mainlatch[2];

//------------------------------------------------------- CPU2 — Sub ----------------------------------------------------------//

wire [15:0] cpu2_A;
wire [7:0]  cpu2_Dout;
wire        cpu2_WR_n, cpu2_RD_n, cpu2_MREQ_n, cpu2_IORQ_n, cpu2_M1_n, cpu2_RFSH_n;

T80pa cpu2
(
	.RESET_n(reset & ~cpu2_rst), .CLK(clk_49m),
	.CEN_p(cen_cpu & ~pause), .CEN_n(~cen_cpu & ~pause),
	.WAIT_n(1'b1),
	.INT_n(~cpu2_irq), .NMI_n(1'b1),
	.M1_n(cpu2_M1_n), .MREQ_n(cpu2_MREQ_n), .IORQ_n(cpu2_IORQ_n),
	.RD_n(cpu2_RD_n), .WR_n(cpu2_WR_n), .RFSH_n(cpu2_RFSH_n),
	.A(cpu2_A), .DI(cpu2_Din), .DO(cpu2_Dout)
);

// Sub IRQ: 4x per frame at (scanline & 0x3F) == 0x20 (scanlines 32, 96, 160, 224)
reg cpu2_irq = 1'b0;
always_ff @(posedge clk_49m) begin
	if (!reset || cpu2_rst) cpu2_irq <= 0;
	else begin
		if (cen_pix && (base_h_cnt == 9'd0) && (v_cnt[5:0] == 6'h20))
			cpu2_irq <= 1;
		if (~cpu2_IORQ_n & ~cpu2_M1_n) cpu2_irq <= 0;
	end
end

// Sub CPU memory decode — variant-aware (kyugo.cpp 484-532)
wire cpu2_mem_valid = ~cpu2_MREQ_n & cpu2_RFSH_n;

// ROM region: Gyrodine 0000-1FFF (8KB), all others 0000-7FFF (16KB+)
wire cs2_rom = cpu2_mem_valid & ((variant_sel == VAR_GYRO)
                                 ? (~cpu2_A[15] & ~cpu2_A[14] & ~cpu2_A[13])
                                 : ~cpu2_A[15]);

// Shared RAM region (per variant)
wire cs2_shared = cpu2_mem_valid &
    ( ((variant_sel == VAR_GYRO)     && (cpu2_A[15:11] == 5'b01000))    // 4000-47FF
    | ((variant_sel == VAR_REPULSE)  && (cpu2_A[15:11] == 5'b10100))    // A000-A7FF
    | ((variant_sel == VAR_FLASHGAL) && (cpu2_A[15:11] == 5'b11100))    // E000-E7FF
    | ((variant_sel == VAR_SRDMISSN) && (cpu2_A[15:11] == 5'b10000))    // 8000-87FF
    | ((variant_sel == VAR_LEGEND)   && (cpu2_A[15:11] == 5'b11000)) ); // C000-C7FF

// SRDMission has an extra 2KB sub-only RAM at 0x8800-0x8FFF
wire cs2_subram = cpu2_mem_valid & (variant_sel == VAR_SRDMISSN) & (cpu2_A[15:11] == 5'b10001);

// Input ports (P1, P2, SYSTEM) — addresses vary per variant
wire cs2_p1, cs2_p2, cs2_system;
assign cs2_system = cpu2_mem_valid &
    ( ((variant_sel == VAR_GYRO)     && (cpu2_A[15:0] == 16'h8080))
    | ((variant_sel == VAR_REPULSE)  && (cpu2_A[15:0] == 16'hC080))
    | ((variant_sel == VAR_FLASHGAL) && (cpu2_A[15:0] == 16'hC040))
    | ((variant_sel == VAR_SRDMISSN) && (cpu2_A[15:0] == 16'hF400))
    | ((variant_sel == VAR_LEGEND)   && (cpu2_A[15:0] == 16'hF800)) );
assign cs2_p1 = cpu2_mem_valid &
    ( ((variant_sel == VAR_GYRO)     && (cpu2_A[15:0] == 16'h8040))
    | ((variant_sel == VAR_REPULSE)  && (cpu2_A[15:0] == 16'hC040))
    | ((variant_sel == VAR_FLASHGAL) && (cpu2_A[15:0] == 16'hC080))
    | ((variant_sel == VAR_SRDMISSN) && (cpu2_A[15:0] == 16'hF401))
    | ((variant_sel == VAR_LEGEND)   && (cpu2_A[15:0] == 16'hF801)) );
assign cs2_p2 = cpu2_mem_valid &
    ( ((variant_sel == VAR_GYRO)     && (cpu2_A[15:0] == 16'h8000))
    | ((variant_sel == VAR_REPULSE)  && (cpu2_A[15:0] == 16'hC000))
    | ((variant_sel == VAR_FLASHGAL) && (cpu2_A[15:0] == 16'hC0C0))
    | ((variant_sel == VAR_SRDMISSN) && (cpu2_A[15:0] == 16'hF402))
    | ((variant_sel == VAR_LEGEND)   && (cpu2_A[15:0] == 16'hF802)) );

// I/O port decode — variant-aware (kyugo.cpp 547-583)
// AY1 base port: Gyro/Repulse = 0x00, Flashgal = 0x40, SRD/Legend = 0x80
// AY2 base port: Gyro = 0xC0, Repulse = 0x40, Flashgal = 0x80, SRD/Legend = 0x84
// Coin counter: Repulse/Flashgal = 0xC0/0xC1, SRD/Legend = 0x90/0x91 (none on Gyrodine)
wire io_active = ~cpu2_IORQ_n & cpu2_M1_n;   // exclude interrupt acknowledge cycles

wire [7:0] ay1_base = (variant_sel == VAR_FLASHGAL) ? 8'h40 :
                      (variant_sel == VAR_SRDMISSN || variant_sel == VAR_LEGEND) ? 8'h80 :
                      8'h00;
wire [7:0] ay2_base = (variant_sel == VAR_GYRO)     ? 8'hC0 :
                      (variant_sel == VAR_REPULSE)  ? 8'h40 :
                      (variant_sel == VAR_FLASHGAL) ? 8'h80 :
                                                      8'h84;   // SRD/Legend
wire [7:0] coin_base = (variant_sel == VAR_REPULSE || variant_sel == VAR_FLASHGAL) ? 8'hC0 :
                       (variant_sel == VAR_SRDMISSN || variant_sel == VAR_LEGEND)  ? 8'h90 :
                                                                                     8'hFF;  // unused for Gyro

wire cs2_ay1_addr = io_active & ~cpu2_WR_n & (cpu2_A[7:0] == ay1_base);
wire cs2_ay1_data = io_active & ~cpu2_WR_n & (cpu2_A[7:0] == (ay1_base | 8'h01));
wire cs2_ay1_rd   = io_active & ~cpu2_RD_n & (cpu2_A[7:0] == (ay1_base | 8'h02));
wire cs2_ay2_addr = io_active & ~cpu2_WR_n & (cpu2_A[7:0] == ay2_base);
wire cs2_ay2_data = io_active & ~cpu2_WR_n & (cpu2_A[7:0] == (ay2_base | 8'h01));
wire cs2_coin_w   = io_active & ~cpu2_WR_n & (variant_sel != VAR_GYRO) &
                    ((cpu2_A[7:0] == coin_base) | (cpu2_A[7:0] == (coin_base | 8'h01)));

// Coin counter (bit 0 of data per MAME coin_counter_w)
reg [1:0] coin_counter_r = 2'd0;
always_ff @(posedge clk_49m) begin
    if (!reset) coin_counter_r <= 2'd0;
    else if (cen_cpu && cs2_coin_w) coin_counter_r[cpu2_A[0]] <= cpu2_Dout[0];
end
assign coin_counter = coin_counter_r;

//---------------------------------------------------------- ROMs -------------------------------------------------------------//

wire [7:0] main_rom_D;
eprom_32k main_rom (.CLK(clk_49m), .ADDR(cpu1_A[14:0]), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(main_rom_cs_i), .WR(ioctl_wr), .DATA(main_rom_D));

wire [7:0] sub_rom_D;
eprom_16k sub_rom (.CLK(clk_49m), .ADDR(cpu2_A[13:0]), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(sub_rom_cs_i), .WR(ioctl_wr), .DATA(sub_rom_D));

//------------------------------------------------------- VRAM (Kyugo map) ---------------------------------------------------//

// bgvideoram 8000-87FF (2KB)
wire [7:0] bgvram_D, bgvram_rD;
reg [10:0] bgvram_raddr;
dpram_dc #(.widthad_a(11)) bgvram (
	.clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
	.wren_a(cs_bgvram & ~cpu1_WR_n), .q_a(bgvram_D),
	.clock_b(clk_49m), .address_b(bgvram_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(bgvram_rD));

// bgattribram 8800-8FFF (2KB)
wire [7:0] bgattr_D, bgattr_rD;
reg [10:0] bgattr_raddr;
dpram_dc #(.widthad_a(11)) bgattr (
	.clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
	.wren_a(cs_bgattr & ~cpu1_WR_n), .q_a(bgattr_D),
	.clock_b(clk_49m), .address_b(bgattr_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(bgattr_rD));

// fgvideoram 9000-97FF (2KB)
wire [7:0] fgvram_D, fgvram_rD;
reg [10:0] fgvram_raddr;
dpram_dc #(.widthad_a(11)) fgvram (
	.clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
	.wren_a(cs_fgvram & ~cpu1_WR_n), .q_a(fgvram_D),
	.clock_b(clk_49m), .address_b(fgvram_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(fgvram_rD));

// spriteram[1] 9800-9FFF (2KB, lower nibble only on CPU reads)
wire [7:0] spram1_D, spram1_rD;
reg [10:0] spram1_raddr;
dpram_dc #(.widthad_a(11)) spram1 (
	.clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
	.wren_a(cs_spram1 & ~cpu1_WR_n), .q_a(spram1_D),
	.clock_b(clk_49m), .address_b(spram1_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(spram1_rD));

// spriteram[0] A000-A7FF (2KB)
wire [7:0] spram0_D, spram0_rD;
reg [10:0] spram0_raddr;
dpram_dc #(.widthad_a(11)) spram0 (
	.clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
	.wren_a(cs_spram0 & ~cpu1_WR_n), .q_a(spram0_D),
	.clock_b(clk_49m), .address_b(spram0_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(spram0_rD));

// Shared RAM (2KB, dual-port). Main side accessed at F000-F7FF (all variants) and also E000-E7FF on SRDMission.
wire [7:0] shared_ram_D_cpu1, shared_ram_D_cpu2;
wire       cs_shared_main = cs_shared | cs_shared_e;
dpram_dc #(.widthad_a(11)) shared_ram (
	.clock_a(clk_49m),
	.address_a(hs_write ? hs_address[10:0] : cpu1_A[10:0]),
	.data_a(hs_write ? hs_data_in : cpu1_Dout),
	.wren_a((cs_shared_main & ~cpu1_WR_n) | hs_write),
	.q_a(shared_ram_D_cpu1),
	.clock_b(clk_49m), .address_b(cpu2_A[10:0]), .data_b(cpu2_Dout),
	.wren_b(cs2_shared & ~cpu2_WR_n), .q_b(shared_ram_D_cpu2));

assign hs_data_out = shared_ram_D_cpu1;

// SRDMission sub-only RAM 0x8800-0x8FFF (2KB)
wire [7:0] subram_D;
spram #(.ADDR_WIDTH(11), .DATA_WIDTH(8)) sub_ram (
	.clk(clk_49m), .addr(cpu2_A[10:0]), .data(cpu2_Dout),
	.we(cs2_subram & ~cpu2_WR_n), .q(subram_D));

// CPU1 data bus mux — spram1 reads return value | 0xF0 (lower nibble only per MAME spriteram_2_r)
wire [7:0] cpu1_Din = cs_rom         ? main_rom_D             :
                      cs_bgvram      ? bgvram_D                :
                      cs_bgattr      ? bgattr_D                :
                      cs_fgvram      ? fgvram_D                :
                      cs_spram1      ? (spram1_D | 8'hF0)     :
                      cs_spram0      ? spram0_D                :
                      cs_shared_main ? shared_ram_D_cpu1       : 8'hFF;

wire [7:0] p1_inputs     = p1_controls;
wire [7:0] p2_inputs     = p2_controls;
wire [7:0] system_inputs = sys_controls;

//---------------------------------------------- Scroll + GFX control registers ------------------------------------------//

reg [7:0] scroll_x_lo = 8'd0;
reg       scroll_x_hi = 1'b0;
reg [7:0] scroll_y_r  = 8'd0;
reg       fgcolor     = 1'b0;
reg       bgpalbank   = 1'b0;

always_ff @(posedge clk_49m) begin
	if (!reset) begin
		scroll_x_lo <= 8'd0; scroll_x_hi <= 1'b0;
		scroll_y_r  <= 8'd0; fgcolor     <= 1'b0; bgpalbank <= 1'b0;
	end else if (cen_cpu) begin
		if (cs_scrollxlo) scroll_x_lo <= cpu1_Dout;
		if (cs_scrolly)   scroll_y_r  <= cpu1_Dout;
		if (cs_gfxctrl) begin
			scroll_x_hi <= cpu1_Dout[0];
			fgcolor     <= cpu1_Dout[5];
			bgpalbank   <= cpu1_Dout[6];
		end
	end
end

//--------------------------------------------------------- AY-3-8910 x2 ---------------------------------------------------//

// AY1: DSW1 → port A, DSW2 → port B
wire [7:0] ay1_dout;
wire [9:0] ay1_sound;
jt49_bus ay1 (
	.rst_n(reset), .clk(clk_49m), .clk_en(cen_ay),
	.bdir(cs2_ay1_addr | cs2_ay1_data), .bc1(cs2_ay1_addr | cs2_ay1_rd),
	.din(cpu2_Dout), .dout(ay1_dout), .sel(1'b1),
	.sound(ay1_sound), .sample(), .A(), .B(), .C(),
	.IOA_in(dip_sw[7:0]), .IOB_in(dip_sw[15:8]), .IOA_out(), .IOB_out()
);

// AY2: sound output only, no input ports
wire [7:0] ay2_dout;
wire [9:0] ay2_sound;
jt49_bus ay2 (
	.rst_n(reset), .clk(clk_49m), .clk_en(cen_ay),
	.bdir(cs2_ay2_addr | cs2_ay2_data), .bc1(cs2_ay2_addr),
	.din(cpu2_Dout), .dout(ay2_dout), .sel(1'b1),
	.sound(ay2_sound), .sample(), .A(), .B(), .C(),
	.IOA_in(8'hFF), .IOB_in(8'hFF), .IOA_out(), .IOB_out()
);

// Mix at 50% each to stay within signed 16-bit range
assign sound = $signed({1'b0, ay1_sound, 4'd0}) + $signed({1'b0, ay2_sound, 4'd0});

wire [7:0] cpu2_Din = (~cpu2_IORQ_n) ? (cs2_ay1_rd ? ay1_dout : 8'hFF) :
                      cs2_rom    ? sub_rom_D         :
                      cs2_shared ? shared_ram_D_cpu2 :
                      cs2_subram ? subram_D          :
                      cs2_p2     ? p2_inputs         :
                      cs2_p1     ? p1_inputs         :
                      cs2_system ? system_inputs     : 8'hFF;

//----------------------------------------------- Graphics ROMs -----------------------------------------------------------//

reg [11:0] fgtile_addr;
wire [7:0] fgtile_D;
eprom_4k fgtile_rom (.CLK(clk_49m), .ADDR(fgtile_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(fg_rom_cs_i), .WR(ioctl_wr), .DATA(fgtile_D));

reg [13:0] bg0_addr, bg1_addr, bg2_addr;
wire [7:0] bg0_D, bg1_D, bg2_D;
eprom_16k bg0_rom (.CLK(clk_49m), .ADDR(bg0_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg0_rom_cs_i), .WR(ioctl_wr), .DATA(bg0_D));
eprom_16k bg1_rom (.CLK(clk_49m), .ADDR(bg1_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg1_rom_cs_i), .WR(ioctl_wr), .DATA(bg1_D));
eprom_16k bg2_rom (.CLK(clk_49m), .ADDR(bg2_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg2_rom_cs_i), .WR(ioctl_wr), .DATA(bg2_D));

reg [14:0] spr_addr;
wire [7:0] spr0_D, spr1_D, spr2_D;
eprom_32k spr0_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr0_rom_cs_i), .WR(ioctl_wr), .DATA(spr0_D));
eprom_32k spr1_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr1_rom_cs_i), .WR(ioctl_wr), .DATA(spr1_D));
eprom_32k spr2_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr2_rom_cs_i), .WR(ioctl_wr), .DATA(spr2_D));

wire [7:0] prom_addr;
reg  [4:0] prom_lut_addr;
wire [7:0] prom_r_D, prom_g_D, prom_b_D;
eprom_256b prom_r_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_r_cs_i), .WR(ioctl_wr), .DATA(prom_r_D));
eprom_256b prom_g_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_g_cs_i), .WR(ioctl_wr), .DATA(prom_g_D));
eprom_256b prom_b_rom (.CLK(clk_49m), .ADDR(prom_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_b_cs_i), .WR(ioctl_wr), .DATA(prom_b_D));

wire [7:0] prom_lut_D;
eprom_32b prom_lut_rom (.CLK(clk_49m), .ADDR(prom_lut_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_lut_cs_i), .WR(ioctl_wr), .DATA(prom_lut_D));

wire [7:0] prom_tim_D;
eprom_32b prom_tim_rom (.CLK(clk_49m), .ADDR(5'd0), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(prom_tim_cs_i), .WR(ioctl_wr), .DATA(prom_tim_D));

//----------------------------------------------- BG render pipeline -------------------------------------------------//

// Screen coords (with flip)
wire [8:0] bg_sx = flip_screen ? (9'd287 - base_h_cnt) : base_h_cnt;
wire [8:0] bg_sy = flip_screen ? (9'd239 - v_cnt)      : v_cnt;

// World coords (BG has set_scrolldx(-32) → +32 in world space when not flipped)
wire [8:0] scroll_x_full = {scroll_x_hi, scroll_x_lo};
wire [9:0] bg_world_x_pre = {1'b0, bg_sx} + {1'b0, scroll_x_full} + 10'd32;
wire [8:0] bg_world_x = bg_world_x_pre[8:0];   // wraps mod 512 (matches 64x8 = 512 BG width)
wire [8:0] bg_world_y = bg_sy + {1'b0, scroll_y_r};

wire [5:0] bg_col = bg_world_x[8:3];   // 0..63
wire [4:0] bg_row = bg_world_y[7:3];   // 0..31
wire [2:0] bg_fx  = bg_world_x[2:0];   // 0..7
wire [2:0] bg_fy  = bg_world_y[2:0];

// "next" = data being fetched for the upcoming 8-pixel tile
// "lat" = data for the currently-displaying 8-pixel tile
reg  [9:0] bg_code_nxt;
reg  [4:0] bg_color_nxt;
reg        bg_fx_invert_nxt;       // 1 = display bit 0 is leftmost (per-tile flipx XOR screen flip)
reg  [2:0] bg_fy_eff;              // fine_y after applying flipy + screen flip
reg  [7:0] bg_p0_nxt, bg_p1_nxt, bg_p2_nxt;

reg  [4:0] bg_color_lat;
reg        bg_fx_invert_lat;
reg  [7:0] bg_p0_lat, bg_p1_lat, bg_p2_lat;

// Pipeline timing on cen_pix ticks within the 8-pixel tile:
//   fx=0: drive bgvram_raddr / bgattr_raddr for NEXT tile (col + 1)
//   fx=1: bgvram_rD / bgattr_rD valid → latch code, color, flip bits, fy_eff; drive plane ROM addrs
//   fx=2: bg0_D / bg1_D / bg2_D valid → latch into _nxt
//   fx=7: promote _nxt → _lat (becomes the displayed tile starting next fx=0)
always_ff @(posedge clk_49m) begin
    if (cen_pix) begin
        case (bg_fx)
            3'd0: begin
                bgvram_raddr <= {bg_row, bg_col + 6'd1};
                bgattr_raddr <= {bg_row, bg_col + 6'd1};
            end
            3'd1: begin
                bg_code_nxt      <= {bgattr_rD[1:0], bgvram_rD};
                bg_color_nxt     <= {bgpalbank, bgattr_rD[7:4]};
                bg_fx_invert_nxt <= bgattr_rD[3] ^ flip_screen;            // per-tile flipx XOR screen flip
                bg_fy_eff        <= bg_fy ^ {3{bgattr_rD[2] ^ flip_screen}}; // flipy XOR screen flip
                bg0_addr <= {1'b0, {bgattr_rD[1:0], bgvram_rD}, (bg_fy ^ {3{bgattr_rD[2] ^ flip_screen}})};
                bg1_addr <= {1'b0, {bgattr_rD[1:0], bgvram_rD}, (bg_fy ^ {3{bgattr_rD[2] ^ flip_screen}})};
                bg2_addr <= {1'b0, {bgattr_rD[1:0], bgvram_rD}, (bg_fy ^ {3{bgattr_rD[2] ^ flip_screen}})};
            end
            3'd2: begin
                bg_p0_nxt <= bg0_D;
                bg_p1_nxt <= bg1_D;
                bg_p2_nxt <= bg2_D;
            end
            3'd7: begin
                bg_color_lat     <= bg_color_nxt;
                bg_fx_invert_lat <= bg_fx_invert_nxt;
                bg_p0_lat        <= bg_p0_nxt;
                bg_p1_lat        <= bg_p1_nxt;
                bg_p2_lat        <= bg_p2_nxt;
            end
            default: ; // idle
        endcase
    end
end

// Display: select bit from latched plane bytes
// Default (no per-tile flipx, no screen flip): bit 7 = leftmost pixel → use ~bg_fx
// With flip: bit 0 = leftmost pixel → use bg_fx
wire [2:0] bg_pix_bit_idx = bg_fx_invert_lat ? bg_fx : ~bg_fx;
wire bg_p0_bit = bg_p0_lat[bg_pix_bit_idx];
wire bg_p1_bit = bg_p1_lat[bg_pix_bit_idx];
wire bg_p2_bit = bg_p2_lat[bg_pix_bit_idx];
wire [2:0] bg_pix = {bg_p2_bit, bg_p1_bit, bg_p0_bit};

wire [7:0] bg_palette_index = {bg_color_lat[4:0], bg_pix[2:0]};

//----------------------------------------------- FG render pipeline -------------------------------------------------//

// FG uses same flipped screen→world mapping as BG; no FG scroll
wire [5:0] fg_col = bg_sx[8:3];
wire [4:0] fg_row = bg_sy[7:3];
wire [2:0] fg_fx  = bg_sx[2:0];
wire [2:0] fg_fy  = bg_sy[2:0];

reg  [7:0] fg_code_nxt;
reg  [4:0] fg_color_nxt;
reg  [7:0] fg_byte_l_nxt, fg_byte_r_nxt;

reg  [4:0] fg_color_lat;
reg  [7:0] fg_byte_l_lat, fg_byte_r_lat;

// Pipeline timing on cen_pix ticks within the 8-pixel tile:
//   fx=0: drive fgvram_raddr for NEXT tile (col + 1)
//   fx=1: latch fg_code_nxt; drive prom_lut_addr from code[7:3]
//   fx=2: latch fg_color_nxt from prom_lut_D; drive fgtile_addr (left byte: y)
//   fx=3: latch fg_byte_l_nxt; drive fgtile_addr (right byte: y+8)
//   fx=4: latch fg_byte_r_nxt
//   fx=7: promote _nxt → _lat
always_ff @(posedge clk_49m) begin
    if (cen_pix) begin
        case (fg_fx)
            3'd0: fgvram_raddr <= {fg_row, fg_col + 6'd1};
            3'd1: begin
                fg_code_nxt   <= fgvram_rD;
                prom_lut_addr <= fgvram_rD[7:3];
            end
            3'd2: begin
                fg_color_nxt <= {prom_lut_D[3:0], fgcolor};
                fgtile_addr  <= {fg_code_nxt, 1'b0, fg_fy};   // left chunk byte at offset y
            end
            3'd3: begin
                fg_byte_l_nxt <= fgtile_D;
                fgtile_addr   <= {fg_code_nxt, 1'b1, fg_fy};   // right chunk byte at offset y+8
            end
            3'd4: fg_byte_r_nxt <= fgtile_D;
            3'd7: begin
                fg_color_lat  <= fg_color_nxt;
                fg_byte_l_lat <= fg_byte_l_nxt;
                fg_byte_r_lat <= fg_byte_r_nxt;
            end
            default: ;
        endcase
    end
end

// FG bit decode: gfxlayout puts plane 0 at bits[0..3] of bit-stream, plane 1 at bits[4..7].
//   In MSB-first byte packing: plane 0 → byte[7..4], plane 1 → byte[3..0].
//   Pixel x[2]=0 selects byte_l (offset y); x[2]=1 selects byte_r (offset y+8).
wire [7:0] fg_byte_sel = fg_fx[2] ? fg_byte_r_lat : fg_byte_l_lat;
wire [1:0] fg_xic      = ~fg_fx[1:0];
wire fg_p0_bit = fg_byte_sel[{1'b1, fg_xic}];   // bit (4 + ~x_in_chunk) = (7 - x_in_chunk)
wire fg_p1_bit = fg_byte_sel[{1'b0, fg_xic}];   // bit (0 + ~x_in_chunk) = (3 - x_in_chunk)
wire [1:0] fg_pix = {fg_p1_bit, fg_p0_bit};

// FG palette index: ((color_codes[code>>3] << 1 | fgcolor) << 2) | pen
//   = {color_codes[3:0], fgcolor, pen[1:0]} (7 bits, padded to 8)
wire [7:0] fg_palette_index = {1'b0, fg_color_lat[4:0], fg_pix[1:0]};

//----------------------------------------------- Sprite engine (Phase 7) -------------------------------------------//

// Shadow fgvram BRAM for sprite area3 reads (FG pipeline owns the main fgvram read port)
wire [7:0]  fgvram_spr_rD;
reg  [10:0] fgvram_spr_raddr;
dpram_dc #(.widthad_a(11)) fgvram_spr (
    .clock_a(clk_49m), .address_a(cpu1_A[10:0]), .data_a(cpu1_Dout),
    .wren_a(cs_fgvram & ~cpu1_WR_n), .q_a(),
    .clock_b(clk_49m), .address_b(fgvram_spr_raddr), .data_b(8'd0), .wren_b(1'b0), .q_b(fgvram_spr_rD));

// Sprite line buffer (double-buffered). Each entry: bit 8 = valid (opaque), bits [7:0] = palette index.
reg [8:0] spr_lb_a [0:511];
reg [8:0] spr_lb_b [0:511];
reg       write_buf;            // 0 → write A / read B; 1 → write B / read A
reg       eol_d;
always_ff @(posedge clk_49m) begin
    if (cen_pix) begin
        eol_d <= (base_h_cnt == 9'd395);
        if (eol_d) write_buf <= ~write_buf;
    end
end

reg       lb_we;
reg [8:0] lb_waddr;
reg [8:0] lb_wdata;
reg [8:0] spr_lb_a_rdata, spr_lb_b_rdata;
always_ff @(posedge clk_49m) begin
    spr_lb_a_rdata <= spr_lb_a[base_h_cnt];
    spr_lb_b_rdata <= spr_lb_b[base_h_cnt];
    if (lb_we && !write_buf) spr_lb_a[lb_waddr] <= lb_wdata;
    if (lb_we &&  write_buf) spr_lb_b[lb_waddr] <= lb_wdata;
end
wire [8:0] spr_lb_rdata = write_buf ? spr_lb_a_rdata : spr_lb_b_rdata;

// Sprite evaluator FSM (clocked by clk_49m; 24 slots × ~24 clk + 288 clr ≈ 864 clk per scanline,
// scanline = ~4224 clk_49m cycles → easily fits)
localparam [3:0] SS_IDLE = 4'd0,  SS_CLR  = 4'd1,  SS_FS0  = 4'd2,  SS_FS1  = 4'd3,
                 SS_FS2  = 4'd4,  SS_FR0  = 4'd5,  SS_FR1  = 4'd6,  SS_FRL  = 4'd7,
                 SS_FRR  = 4'd8,  SS_WPX  = 4'd9,  SS_NEXT = 4'd10;

reg [3:0] spr_st;
reg [4:0] slot_n;
reg [8:0] clr_idx;
reg [3:0] px_idx;
reg [8:0] base_h_prev;

reg [7:0] sy_y_raw;
reg [7:0] x_lo_lat;
reg       x_hi_lat;
reg [4:0] color_lat;
reg [8:0] sy_full;
reg [8:0] sx_full;
reg [3:0] y_in_tile;
reg [9:0] code_lat;
reg       flipx_lat, flipy_lat;
reg [7:0] p0_l, p1_l, p2_l;
reg [7:0] p0_r, p1_r, p2_r;

// Slot offset: offs = 2*(n%12) + 64*(n/12), then base 0x28
wire        slot_div  = (slot_n >= 5'd12);
wire [3:0]  slot_mod  = slot_div ? (slot_n[3:0] - 4'd12) : slot_n[3:0];
wire [10:0] slot_offs = 11'h28 + {6'd0, slot_mod, 1'b0} + (slot_div ? 11'd64 : 11'd0);

// Eval Y: next scanline's screen Y. Visible v_cnt 16..239 → screen Y 0..223. Eval for v_cnt+1.
wire [8:0] eval_y9   = v_cnt - 9'd15;
wire [7:0] eval_y    = eval_y9[7:0];

// MAME sy/sx wrap (sy = (257 - y_raw); if > 240 then -256)
wire [8:0] sy_pre  = 9'd257 - {1'b0, sy_y_raw};
wire [8:0] sy_calc = (sy_pre > 9'd240) ? (sy_pre - 9'd256) : sy_pre;
wire [8:0] sx_pre  = {x_hi_lat, x_lo_lat};
wire [8:0] sx_calc = (sx_pre > 9'd320) ? (sx_pre - 9'd512) : sx_pre;

// Row hit (in SS_FR0): diff = eval_y - sy_full (9-bit two's complement); hit if diff[8:8]=0 (positive)
wire [8:0] hit_diff = {1'b0, eval_y} - sy_full;
wire       hit      = (hit_diff[8] == 1'b0);
wire [3:0] hit_row  = hit_diff[7:4];
wire [3:0] hit_yint = hit_diff[3:0];

// Sprite code/flip from BRAM in SS_FR1 (combinational from BRAM outputs)
wire [9:0] code_now  = {spram1_rD[0], spram1_rD[1], fgvram_spr_rD};
wire       flipx_now = spram1_rD[3];
wire       flipy_now = spram1_rD[2];
wire [3:0] y_eff_now = flipy_now ? (4'd15 - y_in_tile) : y_in_tile;
wire [3:0] y_eff_lat = flipy_lat ? (4'd15 - y_in_tile) : y_in_tile;

// Pixel decode in SS_WPX
wire [3:0] tile_x_eff = flipx_lat ? (4'd15 - px_idx) : px_idx;
wire [7:0] byte0      = tile_x_eff[3] ? p0_r : p0_l;
wire [7:0] byte1      = tile_x_eff[3] ? p1_r : p1_l;
wire [7:0] byte2      = tile_x_eff[3] ? p2_r : p2_l;
wire [2:0] bit_idx    = ~tile_x_eff[2:0];   // 7 - tile_x_eff[2:0]
wire       pix_p0     = byte0[bit_idx];
wire       pix_p1     = byte1[bit_idx];
wire       pix_p2     = byte2[bit_idx];
wire [2:0] spr_pix    = {pix_p2, pix_p1, pix_p0};
wire [7:0] spr_pal_idx = {color_lat[4:0], spr_pix[2:0]};

wire signed [10:0] sx_signed       = {{2{sx_full[8]}}, sx_full};
wire signed [10:0] screen_x_signed = sx_signed + $signed({7'd0, px_idx});
wire               screen_x_in_range = (screen_x_signed >= 11'sd0) && (screen_x_signed < 11'sd288);

always_ff @(posedge clk_49m) begin
    if (!reset) begin
        spr_st      <= SS_IDLE;
        slot_n      <= 5'd0;
        clr_idx     <= 9'd0;
        lb_we       <= 1'b0;
        base_h_prev <= 9'd0;
    end else begin
        lb_we <= 1'b0;
        if (cen_pix) base_h_prev <= base_h_cnt;

        if (cen_pix && (base_h_cnt == 9'd0) && (base_h_prev != 9'd0)) begin
            spr_st  <= SS_CLR;
            slot_n  <= 5'd0;
            clr_idx <= 9'd0;
        end else begin
            case (spr_st)
                SS_IDLE: ;
                SS_CLR: begin
                    lb_we    <= 1'b1;
                    lb_waddr <= clr_idx;
                    lb_wdata <= 9'd0;
                    if (clr_idx == 9'd287) begin
                        clr_idx <= 9'd0;
                        spr_st  <= SS_FS0;
                    end else begin
                        clr_idx <= clr_idx + 9'd1;
                    end
                end
                SS_FS0: begin
                    spram0_raddr     <= slot_offs;
                    fgvram_spr_raddr <= slot_offs + 11'd1;
                    spram1_raddr     <= slot_offs + 11'd1;
                    spr_st <= SS_FS1;
                end
                SS_FS1: begin
                    sy_y_raw     <= spram0_rD;
                    x_lo_lat     <= fgvram_spr_rD;
                    x_hi_lat     <= spram1_rD[0];
                    spram0_raddr <= slot_offs + 11'd1;
                    spr_st       <= SS_FS2;
                end
                SS_FS2: begin
                    color_lat <= spram0_rD[4:0];
                    sy_full   <= sy_calc;
                    sx_full   <= sx_calc;
                    spr_st    <= SS_FR0;
                end
                SS_FR0: begin
                    if (hit) begin
                        y_in_tile        <= hit_yint;
                        fgvram_spr_raddr <= slot_offs + {hit_row, 7'd0};
                        spram1_raddr     <= slot_offs + {hit_row, 7'd0};
                        spr_st           <= SS_FR1;
                    end else begin
                        spr_st <= SS_NEXT;
                    end
                end
                SS_FR1: begin
                    code_lat  <= code_now;
                    flipx_lat <= flipx_now;
                    flipy_lat <= flipy_now;
                    spr_addr  <= {code_now[9:0], y_eff_now[3], 1'b0, y_eff_now[2:0]};   // left half
                    spr_st    <= SS_FRL;
                end
                SS_FRL: begin
                    p0_l <= spr0_D;
                    p1_l <= spr1_D;
                    p2_l <= spr2_D;
                    spr_addr <= {code_lat[9:0], y_eff_lat[3], 1'b1, y_eff_lat[2:0]};    // right half
                    spr_st <= SS_FRR;
                end
                SS_FRR: begin
                    p0_r   <= spr0_D;
                    p1_r   <= spr1_D;
                    p2_r   <= spr2_D;
                    px_idx <= 4'd0;
                    spr_st <= SS_WPX;
                end
                SS_WPX: begin
                    if (screen_x_in_range && (spr_pix != 3'b000)) begin
                        lb_we    <= 1'b1;
                        lb_waddr <= screen_x_signed[8:0];
                        lb_wdata <= {1'b1, spr_pal_idx};
                    end
                    if (px_idx == 4'd15) begin
                        spr_st <= SS_NEXT;
                    end else begin
                        px_idx <= px_idx + 4'd1;
                    end
                end
                SS_NEXT: begin
                    if (slot_n == 5'd23) begin
                        spr_st <= SS_IDLE;
                    end else begin
                        slot_n <= slot_n + 5'd1;
                        spr_st <= SS_FS0;
                    end
                end
                default: spr_st <= SS_IDLE;
            endcase
        end
    end
end

//----------------------------------------------- Composite + palette PROM lookup ------------------------------------//

// Priority: sprite > FG > BG (sprite/FG pen 0 = transparent)
wire       spr_opaque         = spr_lb_rdata[8];
wire [7:0] spr_palette_index  = spr_lb_rdata[7:0];
wire       fg_opaque          = (fg_pix != 2'b00);

wire [7:0] composite_pal = spr_opaque ? spr_palette_index :
                           fg_opaque  ? fg_palette_index  :
                                        bg_palette_index;
assign prom_addr = composite_pal;

// 1-clk_49m latency through palette PROMs. 4-bit channel → 5-bit by replicating MSB.
wire visible = ~hblk & ~vblk;
assign red   = visible ? {prom_r_D[3:0], prom_r_D[3]} : 5'd0;
assign green = visible ? {prom_g_D[3:0], prom_g_D[3]} : 5'd0;
assign blue  = visible ? {prom_b_D[3:0], prom_b_D[3]} : 5'd0;

endmodule
