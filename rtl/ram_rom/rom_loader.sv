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

// ROM layout for Kyugo (index 0) — sized to max across all variant groups.
// MRA pads each region with zeros to the boundary.
// 0x00000-0x07FFF = main_rom  (32KB)
// 0x08000-0x0FFFF = sub_rom   (32KB max, Repulse/Flashgal/SRDMission/Legend)
// 0x10000-0x10FFF = fg_rom    (4KB)
// 0x11000-0x14FFF = bg0_rom   (16KB)
// 0x15000-0x18FFF = bg1_rom   (16KB)
// 0x19000-0x1CFFF = bg2_rom   (16KB)
// 0x1D000-0x24FFF = spr0_rom  (32KB)
// 0x25000-0x2CFFF = spr1_rom  (32KB)
// 0x2D000-0x34FFF = spr2_rom  (32KB)
// 0x35000-0x350FF = prom_r    (256B)
// 0x35100-0x351FF = prom_g    (256B)
// 0x35200-0x352FF = prom_b    (256B)
// 0x35300-0x3531F = prom_lut  (32B)
// 0x35320-0x3533F = prom_tim  (32B)
// Total: 0x35340 ≈ 213KB

module selector
(
    input  logic [24:0] ioctl_addr,
    output logic        main_rom_cs,  // 0x00000-0x07FFF (32KB)
    output logic        sub_rom_cs,   // 0x08000-0x0FFFF (32KB)
    output logic        fg_rom_cs,    // 0x10000-0x10FFF (4KB)
    output logic        bg0_rom_cs,   // 0x11000-0x14FFF (16KB)
    output logic        bg1_rom_cs,   // 0x15000-0x18FFF (16KB)
    output logic        bg2_rom_cs,   // 0x19000-0x1CFFF (16KB)
    output logic        spr0_rom_cs,  // 0x1D000-0x24FFF (32KB)
    output logic        spr1_rom_cs,  // 0x25000-0x2CFFF (32KB)
    output logic        spr2_rom_cs,  // 0x2D000-0x34FFF (32KB)
    output logic        prom_r_cs,    // 0x35000-0x350FF (256B)
    output logic        prom_g_cs,    // 0x35100-0x351FF (256B)
    output logic        prom_b_cs,    // 0x35200-0x352FF (256B)
    output logic        prom_lut_cs,  // 0x35300-0x3531F (32B)
    output logic        prom_tim_cs   // 0x35320-0x3533F (32B)
);
    always_comb begin
        {main_rom_cs, sub_rom_cs, fg_rom_cs, bg0_rom_cs, bg1_rom_cs, bg2_rom_cs,
         spr0_rom_cs, spr1_rom_cs, spr2_rom_cs,
         prom_r_cs, prom_g_cs, prom_b_cs, prom_lut_cs, prom_tim_cs} = 14'd0;

        if      (ioctl_addr < 25'h08000) main_rom_cs = 1;
        else if (ioctl_addr < 25'h10000) sub_rom_cs  = 1;
        else if (ioctl_addr < 25'h11000) fg_rom_cs   = 1;
        else if (ioctl_addr < 25'h15000) bg0_rom_cs  = 1;
        else if (ioctl_addr < 25'h19000) bg1_rom_cs  = 1;
        else if (ioctl_addr < 25'h1D000) bg2_rom_cs  = 1;
        else if (ioctl_addr < 25'h25000) spr0_rom_cs = 1;
        else if (ioctl_addr < 25'h2D000) spr1_rom_cs = 1;
        else if (ioctl_addr < 25'h35000) spr2_rom_cs = 1;
        else if (ioctl_addr < 25'h35100) prom_r_cs   = 1;
        else if (ioctl_addr < 25'h35200) prom_g_cs   = 1;
        else if (ioctl_addr < 25'h35300) prom_b_cs   = 1;
        else if (ioctl_addr < 25'h35320) prom_lut_cs = 1;
        else if (ioctl_addr < 25'h35340) prom_tim_cs = 1;
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
