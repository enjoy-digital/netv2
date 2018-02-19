#include <stdio.h>
#include <generated/csr.h>

#include "mmcm.h"

/*
 * Despite varying pixel clocks, we must keep the PLL VCO operating
 * in the specified range of 600MHz - 1200MHz.
 */
#ifdef CSR_HDMI_OUT0_BASE
void hdmi_out0_mmcm_write(int adr, int data) {
	hdmi_out0_driver_clocking_mmcm_adr_write(adr);
	hdmi_out0_driver_clocking_mmcm_dat_w_write(data);
	hdmi_out0_driver_clocking_mmcm_write_write(1);
	while(!hdmi_out0_driver_clocking_mmcm_drdy_read());
}

int hdmi_out0_mmcm_read(int adr) {
	hdmi_out0_driver_clocking_mmcm_adr_write(adr);
	hdmi_out0_driver_clocking_mmcm_read_write(1);
	while(!hdmi_out0_driver_clocking_mmcm_drdy_read());
	return hdmi_out0_driver_clocking_mmcm_dat_r_read();
}
#endif

#ifdef CSR_HDMI_IN0_BASE
void hdmi_in0_clocking_mmcm_write(int adr, int data) {
	hdmi_in0_clocking_mmcm_adr_write(adr);
	hdmi_in0_clocking_mmcm_dat_w_write(data);
	hdmi_in0_clocking_mmcm_write_write(1);
	while(!hdmi_in0_clocking_mmcm_drdy_read());
}

int hdmi_in0_clocking_mmcm_read(int adr) {
	hdmi_in0_clocking_mmcm_adr_write(adr);
	hdmi_in0_clocking_mmcm_read_write(1);
	while(!hdmi_in0_clocking_mmcm_drdy_read());
	return hdmi_in0_clocking_mmcm_dat_r_read();
}

#ifdef CSR_HDMI_IN0_CLOCKING_MMCM_DRDY_O_ADDR
void hdmi_in0_clocking_mmcm_write_o(int adr, int data) {
	hdmi_in0_clocking_mmcm_adr_write(adr);
	hdmi_in0_clocking_mmcm_dat_w_write(data);
	hdmi_in0_clocking_mmcm_write_o_write(1);
	while(!hdmi_in0_clocking_mmcm_drdy_o_read());
}

int hdmi_in0_clocking_mmcm_read_o(int adr) {
	hdmi_in0_clocking_mmcm_adr_write(adr);
	hdmi_in0_clocking_mmcm_read_o_write(1);
	while(!hdmi_in0_clocking_mmcm_drdy_o_read());
	return hdmi_in0_clocking_mmcm_dat_o_r_read();
}
#endif

static void hdmi_in_0_config_30_60mhz(void) {
	hdmi_in0_clocking_mmcm_write(0x14, 0x1000 | (10<<6) | 10); /* clkfbout_mult  = 20 */
	hdmi_in0_clocking_mmcm_write(0x08, 0x1000 | (10<<6) | 10); /* clkout0_divide = 20 */
	hdmi_in0_clocking_mmcm_write(0x0a, 0x1000 |  (8<<6) |  8); /* clkout1_divide = 16 */
	hdmi_in0_clocking_mmcm_write(0x0c, 0x1000 |  (2<<6) |  2); /* clkout2_divide =  4 */
	hdmi_in0_clocking_mmcm_write(0x0d, 0);                     /* clkout2_divide =  4 */
}

static void hdmi_in_0_config_60_120mhz(void) {
	hdmi_in0_clocking_mmcm_write(0x14, 0x1000 |  (5<<6) | 5); /* clkfbout_mult  = 10 */
	hdmi_in0_clocking_mmcm_write(0x08, 0x1000 |  (5<<6) | 5); /* clkout0_divide = 10 */
	hdmi_in0_clocking_mmcm_write(0x0a, 0x1000 |  (4<<6) | 4); /* clkout1_divide =  8 */
	hdmi_in0_clocking_mmcm_write(0x0c, 0x1000 |  (1<<6) | 1); /* clkout2_divide =  2 */
	hdmi_in0_clocking_mmcm_write(0x0d, 0);                    /* clkout2_divide =  2 */

	// lock/filter parameters derived from these VCO settings for 720p:
	/*      p_BANDWIDTH="OPTIMIZED", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=13.5, #6.734, 7.2
                p_CLKFBOUT_MULT_F=10.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=clk_input_bufr, i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb, */
	hdmi_in0_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in0_clocking_mmcm_write(0x19, (0x0 << 15) | (0x1c << 10) | 0x3001); // lock register 2
	hdmi_in0_clocking_mmcm_write(0x1A, (0x1 << 15) | (0x1c << 10) | 0x33e9); // lock register 3
	hdmi_in0_clocking_mmcm_write(0x4E, (0x1 << 15) | (0x3 << 11) | (0x1 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in0_clocking_mmcm_write(0x4F, (0x2 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8

#ifdef CSR_HDMI_IN0_CLOCKING_MMCM_DRDY_O_ADDR
	hdmi_in0_clocking_mmcm_write_o(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write_o(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write_o(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write_o(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write_o(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in0_clocking_mmcm_write_o(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in0_clocking_mmcm_write_o(0x0d, (1<<6));                /* clkout2_divide = 1 */

	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="LOW", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked_o,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
                p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=mmcm_clk0,  # uncompesated delay for best phase match between master/slave
                i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb_o,  */
	hdmi_in0_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in0_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in0_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in0_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in0_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8
#endif
}

static void hdmi_in_0_config_120_240mhz(void) {
	hdmi_in0_clocking_mmcm_write(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in0_clocking_mmcm_write(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in0_clocking_mmcm_write(0x0d, (1<<6));                /* clkout2_divide = 1 */
	
	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="OPTIMIZED", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked,

	                  # VCO
	  p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
	  p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
	  i_CLKIN1=clk_input_bufr, i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb, */
	hdmi_in0_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in0_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in0_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in0_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in0_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8

#ifdef CSR_HDMI_IN0_CLOCKING_MMCM_DRDY_O_ADDR
	hdmi_in0_clocking_mmcm_write_o(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write_o(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write_o(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in0_clocking_mmcm_write_o(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in0_clocking_mmcm_write_o(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in0_clocking_mmcm_write_o(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in0_clocking_mmcm_write_o(0x0d, (1<<6));                /* clkout2_divide = 1 */

	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="LOW", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked_o,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
                p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=mmcm_clk0,  # uncompesated delay for best phase match between master/slave
                i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb_o,  */
	hdmi_in0_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in0_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in0_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in0_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in0_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8
#endif
}
#endif


#ifdef CSR_HDMI_IN1_BASE
void hdmi_in1_clocking_mmcm_write(int adr, int data) {
	hdmi_in1_clocking_mmcm_adr_write(adr);
	hdmi_in1_clocking_mmcm_dat_w_write(data);
	hdmi_in1_clocking_mmcm_write_write(1);
	while(!hdmi_in1_clocking_mmcm_drdy_read());
}

int hdmi_in1_clocking_mmcm_read(int adr) {
	hdmi_in1_clocking_mmcm_adr_write(adr);
	hdmi_in1_clocking_mmcm_read_write(1);
	while(!hdmi_in1_clocking_mmcm_drdy_read());
	return hdmi_in1_clocking_mmcm_dat_r_read();
}

#ifdef CSR_HDMI_IN1_CLOCKING_MMCM_DRDY_O_ADDR
void hdmi_in1_clocking_mmcm_write_o(int adr, int data) {
	hdmi_in1_clocking_mmcm_adr_write(adr);
	hdmi_in1_clocking_mmcm_dat_w_write(data);
	hdmi_in1_clocking_mmcm_write_o_write(1);
	while(!hdmi_in1_clocking_mmcm_drdy_o_read());
}

int hdmi_in1_clocking_mmcm_read_o(int adr) {
	hdmi_in1_clocking_mmcm_adr_write(adr);
	hdmi_in1_clocking_mmcm_read_o_write(1);
	while(!hdmi_in1_clocking_mmcm_drdy_o_read());
	return hdmi_in1_clocking_mmcm_dat_o_r_read();
}
#endif

static void hdmi_in_1_config_30_60mhz(void) {
	hdmi_in1_clocking_mmcm_write(0x14, 0x1000 | (10<<6) | 10); /* clkfbout_mult  = 20 */
	hdmi_in1_clocking_mmcm_write(0x08, 0x1000 | (10<<6) | 10); /* clkout0_divide = 20 */
	hdmi_in1_clocking_mmcm_write(0x0a, 0x1000 |  (8<<6) |  8); /* clkout1_divide = 16 */
	hdmi_in1_clocking_mmcm_write(0x0c, 0x1000 |  (2<<6) |  2); /* clkout2_divide =  4 */
	hdmi_in1_clocking_mmcm_write(0x0d, 0);                     /* clkout2_divide =  4 */
}

static void hdmi_in_1_config_60_120mhz(void) {
	hdmi_in1_clocking_mmcm_write(0x14, 0x1000 |  (5<<6) | 5); /* clkfbout_mult  = 10 */
	hdmi_in1_clocking_mmcm_write(0x08, 0x1000 |  (5<<6) | 5); /* clkout0_divide = 10 */
	hdmi_in1_clocking_mmcm_write(0x0a, 0x1000 |  (4<<6) | 4); /* clkout1_divide =  8 */
	hdmi_in1_clocking_mmcm_write(0x0c, 0x1000 |  (1<<6) | 1); /* clkout2_divide =  2 */
	hdmi_in1_clocking_mmcm_write(0x0d, 0);                    /* clkout2_divide =  2 */

	// lock/filter parameters derived from these VCO settings for 720p:
	/*      p_BANDWIDTH="OPTIMIZED", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=13.5, #6.734, 7.2
                p_CLKFBOUT_MULT_F=10.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=clk_input_bufr, i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb, */
	hdmi_in1_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in1_clocking_mmcm_write(0x19, (0x0 << 15) | (0x1c << 10) | 0x3001); // lock register 2
	hdmi_in1_clocking_mmcm_write(0x1A, (0x1 << 15) | (0x1c << 10) | 0x33e9); // lock register 3
	hdmi_in1_clocking_mmcm_write(0x4E, (0x1 << 15) | (0x3 << 11) | (0x1 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in1_clocking_mmcm_write(0x4F, (0x2 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8

#ifdef CSR_HDMI_IN1_CLOCKING_MMCM_DRDY_O_ADDR
	hdmi_in1_clocking_mmcm_write_o(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write_o(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write_o(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write_o(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write_o(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in1_clocking_mmcm_write_o(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in1_clocking_mmcm_write_o(0x0d, (1<<6));                /* clkout2_divide = 1 */

	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="LOW", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked_o,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
                p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=mmcm_clk0,  # uncompesated delay for best phase match between master/slave
                i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb_o,  */
	hdmi_in1_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in1_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in1_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in1_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in1_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8
#endif
}

static void hdmi_in_1_config_120_240mhz(void) {
	hdmi_in1_clocking_mmcm_write(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in1_clocking_mmcm_write(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in1_clocking_mmcm_write(0x0d, (1<<6));                /* clkout2_divide = 1 */
	
	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="OPTIMIZED", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked,

	                  # VCO
	  p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
	  p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
	  i_CLKIN1=clk_input_bufr, i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb, */
	hdmi_in1_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in1_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in1_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in1_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in1_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8

#ifdef CSR_HDMI_IN1_CLOCKING_MMCM_DRDY_O_ADDR
	hdmi_in1_clocking_mmcm_write_o(0x14, 0x1000 |  (2<<6) | 3);  /* clkfbout_mult  = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write_o(0x15, 1 << 7);                /* clkfbout_mult  = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write_o(0x08, 0x1000 |  (2<<6) | 3);  /* clkout0_divide = 5 (2/3) */
	hdmi_in1_clocking_mmcm_write_o(0x09, 1 << 7);                /* clkout0_divide = 5 (edge = 1) */
	hdmi_in1_clocking_mmcm_write_o(0x0a, 0x1000 |  (2<<6) | 2);  /* clkout1_divide = 4 */
	hdmi_in1_clocking_mmcm_write_o(0x0c, 0x1000 |  (0<<6) | 0);  /* clkout2_divide = 1 */
	hdmi_in1_clocking_mmcm_write_o(0x0d, (1<<6));                /* clkout2_divide = 1 */

	// lock/filter parameters derived from these VCO settings for 1080p, low bandwidth:
	/*      p_BANDWIDTH="LOW", i_RST=self._mmcm_reset.storage, o_LOCKED=mmcm_locked_o,

                # VCO
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.734,
                p_CLKFBOUT_MULT_F=5.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=mmcm_clk0,  # uncompesated delay for best phase match between master/slave
                i_CLKFBIN=mmcm_fb_o, o_CLKFBOUT=mmcm_fb_o,  */
	hdmi_in1_clocking_mmcm_write(0x18, 0x3e8); // lock register 1
	hdmi_in1_clocking_mmcm_write(0x19, (0x0 << 15) | (0xe << 10) | 0x3801); // lock register 2
	hdmi_in1_clocking_mmcm_write(0x1A, (0x1 << 15) | (0xe << 10) | 0x3be9); // lock register 3
	hdmi_in1_clocking_mmcm_write(0x4E, (0x0 << 15) | (0x1 << 11) | (0x0 << 8) | 0x8); // filter register 1, reserved[7:0] is 0x8
	hdmi_in1_clocking_mmcm_write(0x4F, (0x3 << 11) | (0x2 << 7)  | (0x0 << 4) | 0x8); // filter register 2, reserved[7:0] is 0x8
#endif
}
#endif

void mmcm_config_for_clock(int freq)
{
	/*
	 * FIXME: we also need to configure phase detector
	 */
  
#ifdef CSR_HDMI_IN0_BASE
	if(freq < 3000)
		printf("Frequency too low for input MMCMs\r\n");
	else if(freq < 6000)
		hdmi_in_0_config_30_60mhz();
	else if(freq < 12000)
		hdmi_in_0_config_60_120mhz();
	else if(freq < 24000)
		hdmi_in_0_config_120_240mhz();
	else
		printf("Frequency too high for input MMCMs\r\n");
#endif

#ifdef CSR_HDMI_IN1_BASE
	if(freq < 3000)
		printf("Frequency too low for input MMCMs\r\n");
	else if(freq < 6000)
		hdmi_in_1_config_30_60mhz();
	else if(freq < 12000)
		hdmi_in_1_config_60_120mhz();
	else if(freq < 24000)
		hdmi_in_1_config_120_240mhz();
	else
		printf("Frequency too high for input MMCMs\r\n");
#endif
}


void mmcm_decode_clkreg1(unsigned int data) {
  printf( "  phase mux:0x%x", (data >> 13) & 0x7 );
  printf( " (r):0x%x", (data >> 12) & 0x1 );
  printf( " high time:0x%x", (data >> 6) & 0x3F );
  printf( " low time:0x%x", (data >> 0) & 0x3F );
}

void mmcm_decode_clkreg2(unsigned int data) {
  printf( "  both:" );
  printf( " mx:0x%x", (data >> 8) & 0x3 );
  printf( " edge:0x%x", (data >> 7) & 0x1 );
  printf( " no count:0x%x", (data >> 6) & 0x1 );
  printf( " delay time:0x%x | ", (data >> 0) & 0x3F );

  printf( "  int:" );
  printf( " (r):0x%x", (data >> 15) & 0x1 );
  printf( " frac:0x%x", (data >> 12) & 0x7 );
  printf( " frac_en:0x%x", (data >> 11) & 0x1 );
  printf( " frac_wf_r:0x%x | ", (data >> 10) & 0x1 );
  
  printf( "  frac:" );
  printf( " (r):0x%x", (data >> 14) & 0x3 );
  printf( " phase_mux_f_clkout0:0x%x", (data >> 11) & 0x7 );
  printf( " frac_wf_f_clkout0:0x%x\n", (data >> 10) & 0x1 );
}

void mmcm_decode_divreg(unsigned int data) {
  printf( "  (r):0x%x", (data >> 14) & 0x3 );
  printf( " edge:0x%x", (data >> 13) & 0x1 );
  printf( " no count:0x%x", (data >> 12) & 0x1 );
  printf( " high time:0x%x", (data >> 6) & 0x3f );
  printf( " low time:0x%x\n", (data >> 0) & 0x3f );
}

void mmcm_decode_lockreg1(unsigned int data) {
  printf( "  (r):0x%x", (data >> 10) & 0x3f );
  printf( " lktable[29:0]:0x%x\n", (data >> 0) & 0x3FF );
}

void mmcm_decode_lockreg2(unsigned int data) {
  printf( "  (r):0x%x", (data >> 15) & 0x1 );
  printf( " lktable[34:30]:0x%x", (data >> 10) & 0x1F );
  printf( " lktable[9:0]:0x%x\n", (data >> 0) & 0x3FFF );
}

void mmcm_decode_lockreg3(unsigned int data) {
  printf( "  (r):0x%x", (data >> 15) & 0x1 );
  printf( " lktable[39:35]:0x%x", (data >> 10) & 0x1F );
  printf( " lktable[19:10]:0x%x\n", (data >> 0) & 0x3FFF );
}

void mmcm_decode_filtreg1(unsigned int data) {
  printf( "  table[9]:0x%x", (data >> 15) & 0x1 );
  printf( " (r):0x%x", (data >> 13) & 0x3 );
  printf( " table[8:7]:0x%x", (data >> 11) & 0x3 );
  printf( " (r):0x%x", (data >> 9) & 0x3 );
  printf( " table[6]:0x%x", (data >> 8) & 0x1 );
  printf( " (r):0x%x\n", (data >> 0) & 0xFF );
}

void mmcm_decode_filtreg2(unsigned int data) {
  printf( "  table[5]:0x%x", (data >> 15) & 0x1 );
  printf( " (r):0x%x", (data >> 13) & 0x3 );
  printf( " table[4:3]:0x%x", (data >> 11) & 0x3 );
  printf( " (r):0x%x", (data >> 9) & 0x3 );
  printf( " table[2:1]:0x%x", (data >> 7) & 0x3 );
  printf( " (r):0x%x", (data >> 5) & 0x3 );
  printf( " table[0]:0x%x", (data >> 4) & 0x1 );
  printf( " (r):0x%x\n", (data >> 5) & 0xF );
}

void mmcm_decode_power7(unsigned int data) {
  printf( "  power (must be high):0x%x\n", data & 0xFFFF );
}

void mmcm_decode_reg(unsigned int adr, unsigned int data) {

  switch(adr) {
  case 0x6:
    printf( "\nCLKOUT5 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0x8:
    printf( "\nCLKOUT0 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0xA:
    printf( "\nCLKOUT1 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0xC:
    printf( "\nCLKOUT2 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0xE:
    printf( "\nCLKOUT3 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0x10:
    printf( "\nCLKOUT4 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0x12:
    printf( "\nCLKOUT6 ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
  case 0x14:
    printf( "\nCLKFBOUT ClkReg1: " );
    mmcm_decode_clkreg1(data);
    break;
    
  case 0x7:
    printf( "\nCLKOUT5 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0x9:
    printf( "\nCLKOUT0 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0xB:
    printf( "\nCLKOUT1 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0xD:
    printf( "\nCLKOUT2 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0xF:
    printf( "\nCLKOUT3 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0x11:
    printf( "\nCLKOUT4 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0x13:
    printf( "\nCLKOUT6 ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;
  case 0x15:
    printf( "\nCLKFBOUT ClkReg2: " );
    mmcm_decode_clkreg2(data);
    break;

  case 0x16:
    printf( "\nDivReg: " );
    mmcm_decode_divreg(data);
    break;

  case 0x18:
    printf( "\nLockReg1: " );
    mmcm_decode_lockreg1(data);
    break;

  case 0x19:
    printf( "\nLockReg2: " );
    mmcm_decode_lockreg2(data);
    break;

  case 0x1A:
    printf( "\nLockReg3: " );
    mmcm_decode_lockreg3(data);
    break;

  case 0x28:
    printf( "\nPowerReg 7 series: " );
    mmcm_decode_power7(data);
    break;

  case 0x4E:
    printf( "\nFiltReg1: " );
    mmcm_decode_filtreg1(data);
    break;

  case 0x4F:
    printf( "\nFiltReg2: " );
    mmcm_decode_filtreg2(data);
    break;

  default:
    if( data != 0 )
      printf( " %04x(r)", data );
    else
      printf( " 0(r)", data );
  }
}

void mmcm_dump(void)
{
	int i;
#ifdef CSR_HDMI_OUT0_BASE
	printf("framebuffer MMCM:\r\n");
	for(i=0;i<128;i++)
		printf("%04x ", hdmi_out0_mmcm_read(i));
	printf("\r\n");
#endif
#ifdef CSR_HDMI_IN0_BASE
	printf("dvisampler 0 MMCM:\r\n");
	for(i=0;i<128;i++)
	  mmcm_decode_reg(i, hdmi_in0_clocking_mmcm_read(i));
	printf("\r\n");
#ifdef CSR_HDMI_IN0_CLOCKING_MMCM_DRDY_O_ADDR
	printf("=====================================================\r\n");
	printf("output 0 PLLE2:\r\n");
	for(i=0;i<128;i++)
	  mmcm_decode_reg(i, hdmi_in0_clocking_mmcm_read_o(i));
	printf("\r\n");
#endif
#endif

#ifdef CSR_HDMI_IN1_BASE
	printf("dvisampler 1 MMCM:\r\n");
	for(i=0;i<128;i++)
	  mmcm_decode_reg(i, hdmi_in1_clocking_mmcm_read(i));
	printf("\r\n");
#ifdef CSR_HDMI_IN1_CLOCKING_MMCM_DRDY_O_ADDR
	printf("=====================================================\r\n");
	printf("output 1 PLLE2:\r\n");
	for(i=0;i<128;i++)
	  mmcm_decode_reg(i, hdmi_in1_clocking_mmcm_read_o(i));
	printf("\r\n");
#endif
#endif
	
}
