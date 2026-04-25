//============================================================================
//
//  SD card ROM loader and ROM selector for Kyugo MiSTer.
//  Original framework Copyright (C) 2019, 2020 Kitrinx (aka Rysha)
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

// ROM layout for Kyugo Gyrodine (index 0):
// 0x00000-0x07FFF = main_rom  (maincpu: 4x 8KB, 32KB total)
// 0x08000-0x09FFF = sub_rom   (sub CPU: 1x 8KB)
// 0x0A000-0x0AFFF = fg_rom    (FG chars: 1x 4KB)
// 0x0B000-0x0CFFF = bg0_rom   (BG plane 0: 1x 8KB)
// 0x0D000-0x0EFFF = bg1_rom   (BG plane 1: 1x 8KB)
// 0x0F000-0x10FFF = bg2_rom   (BG plane 2: 1x 8KB)
// 0x11000-0x14FFF = spr0_rom  (sprite plane 0: 2x 8KB = 16KB)
// 0x15000-0x18FFF = spr1_rom  (sprite plane 1: 2x 8KB = 16KB)
// 0x19000-0x1CFFF = spr2_rom  (sprite plane 2: 2x 8KB = 16KB)
// 0x1D000-0x1D0FF = prom_r    (256B)
// 0x1D100-0x1D1FF = prom_g    (256B)
// 0x1D200-0x1D2FF = prom_b    (256B)
// 0x1D300-0x1D31F = prom_lut  (32B char color lookup)
// 0x1D320-0x1D33F = prom_tim  (32B timing, unused)
// Total: 0x1D340 = 119,616 bytes

module selector
(
    input  logic [24:0] ioctl_addr,
    output logic        main_rom_cs,  // 0x00000-0x07FFF (32KB)
    output logic        sub_rom_cs,   // 0x08000-0x09FFF (8KB)
    output logic        fg_rom_cs,    // 0x0A000-0x0AFFF (4KB)
    output logic        bg0_rom_cs,   // 0x0B000-0x0CFFF (8KB)
    output logic        bg1_rom_cs,   // 0x0D000-0x0EFFF (8KB)
    output logic        bg2_rom_cs,   // 0x0F000-0x10FFF (8KB)
    output logic        spr0_rom_cs,  // 0x11000-0x14FFF (16KB)
    output logic        spr1_rom_cs,  // 0x15000-0x18FFF (16KB)
    output logic        spr2_rom_cs,  // 0x19000-0x1CFFF (16KB)
    output logic        prom_r_cs,    // 0x1D000-0x1D0FF (256B)
    output logic        prom_g_cs,    // 0x1D100-0x1D1FF (256B)
    output logic        prom_b_cs,    // 0x1D200-0x1D2FF (256B)
    output logic        prom_lut_cs,  // 0x1D300-0x1D31F (32B)
    output logic        prom_tim_cs   // 0x1D320-0x1D33F (32B)
);
    always_comb begin
        {main_rom_cs, sub_rom_cs, fg_rom_cs, bg0_rom_cs, bg1_rom_cs, bg2_rom_cs,
         spr0_rom_cs, spr1_rom_cs, spr2_rom_cs,
         prom_r_cs, prom_g_cs, prom_b_cs, prom_lut_cs, prom_tim_cs} = 14'd0;

        if      (ioctl_addr < 25'h08000) main_rom_cs = 1;
        else if (ioctl_addr < 25'h0A000) sub_rom_cs  = 1;
        else if (ioctl_addr < 25'h0B000) fg_rom_cs   = 1;
        else if (ioctl_addr < 25'h0D000) bg0_rom_cs  = 1;
        else if (ioctl_addr < 25'h0F000) bg1_rom_cs  = 1;
        else if (ioctl_addr < 25'h11000) bg2_rom_cs  = 1;
        else if (ioctl_addr < 25'h15000) spr0_rom_cs = 1;
        else if (ioctl_addr < 25'h19000) spr1_rom_cs = 1;
        else if (ioctl_addr < 25'h1D000) spr2_rom_cs = 1;
        else if (ioctl_addr < 25'h1D100) prom_r_cs   = 1;
        else if (ioctl_addr < 25'h1D200) prom_g_cs   = 1;
        else if (ioctl_addr < 25'h1D300) prom_b_cs   = 1;
        else if (ioctl_addr < 25'h1D320) prom_lut_cs = 1;
        else if (ioctl_addr < 25'h1D340) prom_tim_cs = 1;
    end
endmodule

////////////
// EPROMS //
////////////

// Main CPU ROM — 32KB (15-bit address)
module eprom_32k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [14:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(15)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[14:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[14:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// 16KB ROM module (14-bit address) — sprite planes
module eprom_16k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [13:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(14)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[13:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[13:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// Generic 8KB ROM module (13-bit address)
module eprom_8k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [12:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(13)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[12:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[12:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// 4KB ROM module (12-bit address) — FG chars
module eprom_4k
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [11:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(12)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[11:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[11:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// 256-byte ROM module (8-bit address) — color PROMs
module eprom_256b
(
    input  logic       CLK,
    input  logic       CLK_DL,
    input  logic [7:0] ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(8)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[7:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[7:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule

// 32-byte ROM module (5-bit address) — char color lookup and timing PROMs
module eprom_32b
(
    input  logic        CLK,
    input  logic        CLK_DL,
    input  logic [4:0]  ADDR,
    input  logic [24:0] ADDR_DL,
    input  logic [7:0]  DATA_IN,
    input  logic        CS_DL,
    input  logic        WR,
    output logic [7:0]  DATA
);
    dpram_dc #(.widthad_a(5)) rom
    (
        .clock_a(CLK),
        .address_a(ADDR[4:0]),
        .q_a(DATA[7:0]),
        .clock_b(CLK_DL),
        .address_b(ADDR_DL[4:0]),
        .data_b(DATA_IN),
        .wren_b(WR & CS_DL)
    );
endmodule
