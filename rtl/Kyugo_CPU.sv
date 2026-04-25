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
	input         pause,
	input  [15:0] hs_address,
	input   [7:0] hs_data_in,
	output  [7:0] hs_data_out,
	input         hs_write
);

//------------------------------------------------------- Clock enables -------------------------------------------------------//

// Pixel clock: 49.152 MHz * 3/32 = 4.608 MHz (Phase 5 will correct to 6.144 MHz = XTAL/3)
wire [1:0] pix_cen_o;
jtframe_frac_cen #(2) pix_cen (.clk(clk_49m), .n(10'd3), .m(10'd32), .cen(pix_cen_o), .cenb());
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

//-------------------------------------------------- CPU1 Address Decoding (Kyugo main map) ----------------------------//

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
wire cs_watchdog  = cpu1_mem_valid & (cpu1_A[15:0] == 16'hE000);               // E000 (Gyrodine watchdog)
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

// Sub CPU memory decode (Gyrodine): ROM 0000-1FFF, shared 4000-47FF, P2@8000, P1@8040, SYS@8080
wire cpu2_mem_valid = ~cpu2_MREQ_n & cpu2_RFSH_n;
wire cs2_rom    = cpu2_mem_valid & ~cpu2_A[15] & ~cpu2_A[14] & ~cpu2_A[13];
wire cs2_shared = cpu2_mem_valid & (cpu2_A[15:11] == 5'b01000);
wire cs2_p2     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8000);
wire cs2_p1     = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8040);
wire cs2_system = cpu2_mem_valid & (cpu2_A[15:0] == 16'h8080);
// AY1 at I/O ports 0x00-0x02; AY2 at I/O ports 0xC0-0xC1 (global_mask 0xFF)
wire cs2_ay1_addr = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[7:0] == 8'h00);
wire cs2_ay1_data = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[7:0] == 8'h01);
wire cs2_ay1_rd   = ~cpu2_IORQ_n & ~cpu2_RD_n & (cpu2_A[7:0] == 8'h02);
wire cs2_ay2_addr = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[7:0] == 8'hC0);
wire cs2_ay2_data = ~cpu2_IORQ_n & ~cpu2_WR_n & (cpu2_A[7:0] == 8'hC1);

//---------------------------------------------------------- ROMs -------------------------------------------------------------//

wire [7:0] main_rom_D;
eprom_32k main_rom (.CLK(clk_49m), .ADDR(cpu1_A[14:0]), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(main_rom_cs_i), .WR(ioctl_wr), .DATA(main_rom_D));

wire [7:0] sub_rom_D;
eprom_8k sub_rom (.CLK(clk_49m), .ADDR(cpu2_A[12:0]), .CLK_DL(clk_49m),
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

// Shared RAM F000-F7FF (2KB, dual-port)
wire [7:0] shared_ram_D_cpu1, shared_ram_D_cpu2;
dpram_dc #(.widthad_a(11)) shared_ram (
	.clock_a(clk_49m),
	.address_a(hs_write ? hs_address[10:0] : cpu1_A[10:0]),
	.data_a(hs_write ? hs_data_in : cpu1_Dout),
	.wren_a((cs_shared & ~cpu1_WR_n) | hs_write),
	.q_a(shared_ram_D_cpu1),
	.clock_b(clk_49m), .address_b(cpu2_A[10:0]), .data_b(cpu2_Dout),
	.wren_b(cs2_shared & ~cpu2_WR_n), .q_b(shared_ram_D_cpu2));

assign hs_data_out = shared_ram_D_cpu1;

// CPU1 data bus mux — spram1 reads return value | 0xF0 (lower nibble only per MAME spriteram_2_r)
wire [7:0] cpu1_Din = cs_rom    ? main_rom_D             :
                      cs_bgvram ? bgvram_D                :
                      cs_bgattr ? bgattr_D                :
                      cs_fgvram ? fgvram_D                :
                      cs_spram1 ? (spram1_D | 8'hF0)     :
                      cs_spram0 ? spram0_D                :
                      cs_shared ? shared_ram_D_cpu1       : 8'hFF;

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
                      cs2_rom ? sub_rom_D : cs2_shared ? shared_ram_D_cpu2 :
                      cs2_p2 ? p2_inputs : cs2_p1 ? p1_inputs :
                      cs2_system ? system_inputs : 8'hFF;

//----------------------------------------------- Graphics ROMs -----------------------------------------------------------//

reg [11:0] fgtile_addr;
wire [7:0] fgtile_D;
eprom_4k fgtile_rom (.CLK(clk_49m), .ADDR(fgtile_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(fg_rom_cs_i), .WR(ioctl_wr), .DATA(fgtile_D));

reg [12:0] bg0_addr, bg1_addr, bg2_addr;
wire [7:0] bg0_D, bg1_D, bg2_D;
eprom_8k bg0_rom (.CLK(clk_49m), .ADDR(bg0_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg0_rom_cs_i), .WR(ioctl_wr), .DATA(bg0_D));
eprom_8k bg1_rom (.CLK(clk_49m), .ADDR(bg1_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg1_rom_cs_i), .WR(ioctl_wr), .DATA(bg1_D));
eprom_8k bg2_rom (.CLK(clk_49m), .ADDR(bg2_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(bg2_rom_cs_i), .WR(ioctl_wr), .DATA(bg2_D));

reg [13:0] spr_addr;
wire [7:0] spr0_D, spr1_D, spr2_D;
eprom_16k spr0_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr0_rom_cs_i), .WR(ioctl_wr), .DATA(spr0_D));
eprom_16k spr1_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr1_rom_cs_i), .WR(ioctl_wr), .DATA(spr1_D));
eprom_16k spr2_rom (.CLK(clk_49m), .ADDR(spr_addr), .CLK_DL(clk_49m),
	.ADDR_DL(ioctl_addr), .DATA_IN(ioctl_data), .CS_DL(spr2_rom_cs_i), .WR(ioctl_wr), .DATA(spr2_D));

wire [7:0] prom_addr = 8'd0;    // Phase 6 drives this
wire [4:0] prom_lut_addr = 5'd0; // Phase 6 drives this
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

//----------------------------------------------- Video output stub (Phases 5-7 implement rendering) -----------------//

assign red   = 5'd0;
assign green = 5'd0;
assign blue  = 5'd0;

endmodule
