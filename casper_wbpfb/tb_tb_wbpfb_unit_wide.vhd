--------------------------------------------------------------------------------
--
-- Copyright (C) 2016
-- ASTRON (Netherlands Institute for Radio Astronomy) <http://www.astron.nl/>
-- P.O.Box 2, 7990 AA Dwingeloo, The Netherlands
--
-- This program is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program.  If not, see <http://www.gnu.org/licenses/>.
--
--------------------------------------------------------------------------------

-- Purpose: Multi-testbench for wpfb_unit_wide using file data
-- Description:
--   Verify wpfb_unit_wide using and data generated by Matlab scripts:
--
--   - $RADIOHDL_WORK/applications/apertif/matlab/run_pfb.m
--   - $RADIOHDL_WORK/applications/apertif/matlab/run_pfb_complex.m
--
-- Usage:
--   > as 4
--   > run -all

LIBRARY IEEE, common_pkg_lib, casper_filter_lib, r2sdf_fft_lib;
USE IEEE.std_logic_1164.ALL;
USE common_pkg_lib.common_pkg.all;
USE casper_filter_lib.fil_pkg.all;
USE r2sdf_fft_lib.rTwoSDFPkg.all;
USE work.wbpfb_gnrcs_intrfcs_pkg.all;
              
ENTITY tb_tb_wbpfb_unit_wide IS
END tb_tb_wbpfb_unit_wide;

ARCHITECTURE tb OF tb_tb_wbpfb_unit_wide IS
  
  constant c_stage_dat_extra_w         : natural := c_dsp_mult_w + 10;
  constant c_nof_blk_per_sync          : natural := 20;
  
  -- wb 1, two real
  CONSTANT c_wb1_two_real_1024        : t_wpfb := (1, 1024, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync, 
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_two_real             : t_wpfb := (1, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                    
  CONSTANT c_wb1_two_real_4streams    : t_wpfb := (1, 32, 0, 4,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  CONSTANT c_wb1_two_real_4channels   : t_wpfb := (1, 32, 2, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  -- wb 4, two real
  CONSTANT c_wb4_two_real_1024        : t_wpfb := (4, 1024, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_stage_dat_extra_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_two_real             : t_wpfb := (4, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                    
  CONSTANT c_wb4_two_real_4streams     : t_wpfb := (4, 32, 0, 4,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                    
  CONSTANT c_wb4_two_real_4channels   : t_wpfb := (4, 32, 2, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, true, 16, 16, 1, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  -- wb 1, complex reordered
  CONSTANT c_wb1_complex_1024         : t_wpfb := (1, 1024, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_complex_64           : t_wpfb := (1, 64, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_complex              : t_wpfb := (1, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_complex_4streams     : t_wpfb := (1, 32, 0, 4,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  CONSTANT c_wb1_complex_4channels    : t_wpfb := (1, 32, 2, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  -- wb 1, complex fft_shift
  CONSTANT c_wb1_complex_fft_shift    : t_wpfb := (1, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true,  true, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  -- wb 1, complex without reorder
  CONSTANT c_wb1_complex_flipped_1024 : t_wpfb := (1, 1024, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_complex_flipped_64   : t_wpfb := (1, 64, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb1_complex_flipped      : t_wpfb := (1, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                    
  -- wb 4, complex reordered
  CONSTANT c_wb4_complex_1024         : t_wpfb := (4, 1024, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_complex_64           : t_wpfb := (4, 64, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_complex              : t_wpfb := (4, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_complex_4streams     : t_wpfb := (4, 32, 0, 4,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  CONSTANT c_wb4_complex_4channels    : t_wpfb := (4, 32, 2, 1,
                                                   16, 1, 8, 16, 16,
                                                   true, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  -- wb 4, complex fft_shift
  CONSTANT c_wb4_complex_fft_shift    : t_wpfb := (4, 32, 0, 1,
                                                   16, 1, 8, 16, 16,
                                                   true,  true, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                   c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);

  -- wb 4, complex without reorder
  CONSTANT c_wb4_complex_flipped_1024     : t_wpfb := (4, 1024, 0, 1,
                                                       16, 1, 8, 16, 16,
                                                       false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                       c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_complex_flipped_64       : t_wpfb := (4, 64, 0, 1,
                                                       16, 1, 8, 16, 16,
                                                       false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                       c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
  CONSTANT c_wb4_complex_flipped          : t_wpfb := (4, 32, 0, 1,
                                                       16, 1, 8, 16, 16,
                                                       false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                       c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                       
  CONSTANT c_wb4_complex_flipped_channels : t_wpfb := (4, 32, 2, 1,
                                                       16, 1, 8, 16, 16,
                                                       false, false, false, 16, 16, 0, c_dsp_mult_w, 18, 8, 2, true, 56, 2, c_nof_blk_per_sync,
                                                       c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
                                                   
  CONSTANT c_dm_1                : natural := 1;  -- diff margin (for stage_dat_w >> c_dsp_mult_w)
  CONSTANT c_dm_3                : natural := 3;  -- diff margin (for 32 point dm=2 appears sufficient, for 1024 point dm=3 is sufficient)
  CONSTANT c_dm_5                : natural := 5;  -- diff margin (for 32 point dm=2 appears sufficient, for 1024 point dm=3 is sufficient)
  
  CONSTANT c_pre_ab              : string := "../../../../../data/mem/hex/run_pfb_m_pfir_coeff_fircls1_16taps_32points_16b";                 -- original version
  CONSTANT c_pre_ab_1024         : string := "../../../../../data/mem/hex/run_pfb_m_pfir_coeff_fircls1_16taps_1024points_16b";                 -- original version
  CONSTANT c_pre_ab_v2           : string := "../../../../../data/mem/hex/run_pfb_m_v2_pfir_coeff_fircls1_16taps_1024points_16b";              -- next version
  CONSTANT c_pre_c               : string := "../../../../../data/mem/hex/run_pfb_complex_m_pfir_coeff_fircls1_16taps_32points_16b";
  CONSTANT c_pre_c_64            : string := "../../../../../data/mem/hex/run_pfb_complex_m_pfir_coeff_fircls1_16taps_64points_16b";
  CONSTANT c_pre_c_1024          : string := "../../../../../data/mem/hex/run_pfb_complex_m_pfir_coeff_fircls1_16taps_1024points_16b";
  
  -- Real input
  CONSTANT c_sinusoid_chirp_1024 : string := "../../../../../data/run_pfb_m_sinusoid_chirp_8b_16taps_1024points_16b.dat";   -- 204800 lines
  CONSTANT c_sinusoid_chirp      : string := "../../../../../data/run_pfb_m_sinusoid_chirp_8b_16taps_32points_16b.dat";     --   6400 lines
  CONSTANT c_sinusoid_1024       : string := "../../../../../data/run_pfb_m_sinusoid_8b_16taps_1024points_16b.dat";         --  51200 lines
  CONSTANT c_sinusoid_1024_v2    : string := "../../../../../data/run_pfb_m_v2_sinusoid_8b_16taps_1024points_16b.dat";      --  51200 lines
  CONSTANT c_sinusoid            : string := "../../../../../data/run_pfb_m_sinusoid_8b_16taps_32points_16b.dat";           --   1600 lines
  CONSTANT c_impulse_chirp       : string := "../../../../../data/run_pfb_m_impulse_chirp_8b_16taps_32points_16b.dat";      --   6400 lines
  CONSTANT c_noise_1024          : string := "../../../../../data/run_pfb_m_noise_8b_16taps_1024points_16b.dat";            --  51200 lines
  CONSTANT c_noise               : string := "../../../../../data/run_pfb_m_noise_8b_16taps_32points_16b.dat";              --   1600 lines
  CONSTANT c_dc_agwn             : string := "../../../../../data/run_pfb_m_dc_agwn_8b_16taps_32points_16b.dat";            --   1600 lines
  -- Complex input
  CONSTANT c_phasor_chirp_1024   : string := "../../../../../data/run_pfb_complex_m_phasor_chirp_8b_16taps_1024points_16b.dat";   -- 204800 lines
  CONSTANT c_phasor_chirp_128    : string := "../../../../../data/run_pfb_complex_m_phasor_chirp_8b_16taps_128points_16b.dat";    --  25600 lines
  CONSTANT c_phasor_chirp_64     : string := "../../../../../data/run_pfb_complex_m_phasor_chirp_8b_16taps_64points_16b.dat";     --  12800 lines
  CONSTANT c_phasor_chirp        : string := "../../../../../data/run_pfb_complex_m_phasor_chirp_8b_16taps_32points_16b.dat";     --   6400 lines
  CONSTANT c_phasor              : string := "../../../../../data/run_pfb_complex_m_phasor_8b_16taps_32points_16b.dat";           --   1600 lines
  CONSTANT c_noise_complex_1024  : string := "../../../../../data/run_pfb_complex_m_noise_complex_8b_16taps_1024points_16b.dat";  --  51200 lines
  CONSTANT c_noise_complex_128   : string := "../../../../../data/run_pfb_complex_m_noise_complex_8b_16taps_128points_16b.dat";   --   6400 lines
  CONSTANT c_noise_complex_64    : string := "../../../../../data/run_pfb_complex_m_noise_complex_8b_16taps_64points_16b.dat";    --   3200 lines
  CONSTANT c_noise_complex       : string := "../../../../../data/run_pfb_complex_m_noise_complex_8b_16taps_32points_16b.dat";    --   1600 lines
  -- Zero input
  CONSTANT c_zero                : string := "UNUSED";  -- zero's data
  CONSTANT c_un                  : string := "UNUSED";  -- zero's data
 
  SIGNAL tb_end : STD_LOGIC := '0';  -- declare tb_end to avoid 'No objects found' error on 'when -label tb_end'
  
BEGIN

-- -- DUT generics
-- g_wpfb : t_wpfb := (4, 32, 0, 1,
--                     16, 1, 8, 16, 16,
--                     false, false, false, 16, 16, 0, c_dsp_mult_w, 2, true, 56, 2, 800000, 
--                     c_fft_pipeline, c_fft_pipeline, c_fil_ppf_pipeline);
-- --  type t_wpfb is record  
-- --    -- General parameters for the wideband poly phase filter
-- --    wb_factor         : natural;        -- = default 4, wideband factor
-- --    nof_points        : natural;        -- = 1024, N point FFT (Also the number of subbands for the filter part)
-- --    nof_chan          : natural;        -- = default 0, defines the number of channels (=time-multiplexed input signals): nof channels = 2**nof_chan     
-- --    nof_wb_streams    : natural;        -- = 1, the number of parallel wideband streams. The filter coefficients are shared on every wb-stream. 
-- --    
-- --    -- Parameters for the poly phase filter
-- --    nof_taps          : natural;        -- = 16, the number of FIR taps per subband
-- --    fil_backoff_w     : natural;        -- = 0, number of bits for input backoff to avoid output overflow
-- --    fil_in_dat_w      : natural;        -- = 8, number of input bits
-- --    fil_out_dat_w     : natural;        -- = 16, number of output bits
-- --    coef_dat_w        : natural;        -- = 16, data width of the FIR coefficients
-- --                                      
-- --    -- Parameters for the FFT         
-- --    use_reorder       : boolean;        -- = false for bit-reversed output, true for normal output
-- --    use_fft_shift     : boolean;        -- = false for [0, pos, neg] bin frequencies order, true for [neg, 0, pos] bin frequencies order in case of complex input
-- --    use_separate      : boolean;        -- = false for complex input, true for two real inputs
-- --    fft_in_dat_w      : natural;        -- = 16, number of input bits
-- --    fft_out_dat_w     : natural;        -- = 13, number of output bits
-- --    fft_out_gain_w    : natural;        -- = 0, output gain factor applied after the last stage output, before requantization to out_dat_w
-- --    stage_dat_w       : natural;        -- = 18, number of bits that are used inter-stage
-- --    guard_w           : natural;        -- = 2
-- --    guard_enable      : boolean;        -- = true
-- --    
-- --    -- Parameters for the statistics
-- --    stat_data_w       : positive;       -- = 56
-- --    stat_data_sz      : positive;       -- = 2
-- --    nof_blk_per_sync  : natural;        -- = 800000, number of FFT output blocks per sync interval
-- -- 
-- --    -- Pipeline parameters for both poly phase filter and FFT. These are heritaged from the filter and fft libraries.  
-- --    pft_pipeline      : t_fft_pipeline;     -- Pipeline settings for the pipelined FFT
-- --    fft_pipeline      : t_fft_pipeline;     -- Pipeline settings for the parallel FFT
-- --    fil_pipeline      : t_fil_ppf_pipeline; -- Pipeline settings for the filter units 
-- --  end record;
-- 
-- -- TB generics
-- g_diff_margin           : integer := 2;  -- maximum difference between HDL output and expected output (> 0 to allow minor rounding differences)
-- 
-- -- PFIR coefficients
-- g_coefs_file_prefix_ab    : string := "data/run_pfb_m_pfir_coeff_fircls1";
-- g_coefs_file_prefix_c     : string := "data/run_pfb_complex_m_pfir_coeff_fircls1";
-- 
-- -- Two real input data files A and B used when g_fft.use_separate = true
-- -- * 1024 points = 512 subbands
-- --g_data_file_a           : string := "data/run_pfb_m_sinusoid_chirp_8b_16taps_1024points_16b_16b.dat";
-- --g_data_file_a_nof_lines : natural := 204800;
-- --g_data_file_b           : string := "data/run_pfb_m_noise_8b_16taps_1024points_16b_16b.dat";
-- --g_data_file_b_nof_lines : natural := 51200;
-- --g_data_file_b           : string := "UNUSED";
-- --g_data_file_b_nof_lines : natural := 0;
-- 
-- -- * 32 points = 16 subbands
-- g_data_file_a           : string := "data/run_pfb_m_sinusoid_chirp_8b_16taps_32points_16b_16b.dat";
-- g_data_file_a_nof_lines : natural := 6400;
-- --g_data_file_a           : string := "data/run_pfb_m_sinusoid_8b_16taps_32points_16b_16b.dat";
-- --g_data_file_a_nof_lines : natural := 160;
-- 
-- --g_data_file_b           : string := "data/run_pfb_m_impulse_chirp_8b_16taps_32points_16b_16b.dat";
-- --g_data_file_b_nof_lines : natural := 6400;
-- g_data_file_b           : string := "UNUSED";
-- g_data_file_b_nof_lines : natural := 0;
-- 
-- -- One complex input data file C used when g_fft.use_separate = false
-- -- * 64 points = 64 channels
-- --g_data_file_c           : string := "data/run_pfb_complex_m_phasor_chirp_8b_16taps_64points_16b_16b.dat";
-- --g_data_file_c_nof_lines : natural := 12800;
-- --g_data_file_c           : string := "data/run_pfb_complex_m_phasor_8b_16taps_64points_16b_16b.dat";
-- --g_data_file_c_nof_lines : natural := 320;
-- --g_data_file_c           : string := "data/run_pfb_complex_m_noise_8b_16taps_64points_16b_16b.dat";
-- --g_data_file_c_nof_lines : natural := 640;
--
-- -- * 32 points = 32 channels
-- --g_data_file_c           : string := "data/run_pfb_complex_m_phasor_chirp_8b_16taps_32points_16b_16b.dat";
-- --g_data_file_c_nof_lines : natural := 6400;
-- --g_data_file_c           : string := "data/run_pfb_complex_m_phasor_8b_16taps_32points_16b_16b.dat";
-- --g_data_file_c_nof_lines : natural := 1600;
-- g_data_file_c           : string := "data/run_pfb_complex_m_noise_complex_8b_16taps_32points_16b_16b.dat";
-- g_data_file_c_nof_lines : natural := 1600;
-- 
-- g_data_file_nof_lines   : natural := 1600;   -- actual number of lines with input data to simulate from the data files, must be <= g_data_file_*_nof_lines
-- g_enable_in_val_gaps    : boolean := FALSE   -- when false then in_val flow control active continuously, else with random inactive gaps
  
  -- Two real input data A and B
  -- * 1024 point (as in Apertif subband filterbank)
--  u_act_wb4_two_real_a0_1024    : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_two_real_1024, c_dm_1, c_pre_ab_v2, c_pre_c_1024, c_sinusoid_1024_v2, 51200, c_zero,   51200, c_un, 0, 51200, FALSE, c_twid_file_stem);
--  u_act_wb4_two_real_ab_1024    : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_two_real_1024, c_dm_1, c_pre_ab_1024, c_pre_c_1024, c_sinusoid_chirp_1024, 204800, c_noise_1024,   51200, c_un, 0, 51200, FALSE, c_twid_file_stem);
--  u_act_wb1_two_real_ab_1024    : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real_1024, c_dm_5, c_pre_ab_1024, c_pre_c_1024, c_sinusoid_chirp_1024, 204800, c_noise_1024,   51200, c_un, 0, 51200, FALSE, c_twid_file_stem);
--  u_act_wb1_two_real_chirp_1024 : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real_1024, c_dm_5, c_pre_ab_1024, c_pre_c_1024, c_sinusoid_chirp_1024, 204800, c_zero,         51200, c_un, 0, 51200, FALSE, c_twid_file_stem);
  
----  -- * 32 point
--  u_act_wb1_two_real_chirp          : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real,           c_dm_5, c_pre_ab, c_pre_c, c_sinusoid_chirp,        6400, c_impulse_chirp, 6400, c_un, 0,  6400, FALSE, c_twid_file_stem);
--  u_act_wb1_two_real_a0             : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real,           c_dm_5, c_pre_ab, c_pre_c, c_zero,                  6400, c_impulse_chirp, 6400, c_un, 0,  6400, FALSE, c_twid_file_stem);
--  u_act_wb1_two_real_b0             : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real,           c_dm_5, c_pre_ab, c_pre_c, c_sinusoid_chirp,        6400, c_zero,          6400, c_un, 0,  6400, FALSE, c_twid_file_stem);
--  u_rnd_wb4_two_real_noise          : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_two_real,           c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
--  u_rnd_wb4_two_real_noise_channels : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_two_real_4channels, c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
--  u_rnd_wb4_two_real_noise_streams  : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_two_real_4streams,  c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
--  u_rnd_wb1_two_real_noise          : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real,           c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
--  u_rnd_wb1_two_real_noise_channels : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real_4channels, c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
--  u_rnd_wb1_two_real_noise_streams  : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_two_real_4streams,  c_dm_5, c_pre_ab, c_pre_c, c_noise,                 1600, c_dc_agwn,       1600, c_un, 0,  1600, TRUE, c_twid_file_stem);
  
  -- Complex input data
  -- * 1024 point
  u_act_wb1_complex_chirp_1024  : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_1024,  c_dm_3, c_pre_ab_1024, c_pre_c_1024, c_un, 0, c_un, 0, c_phasor_chirp_1024,  204800, 51200, FALSE, c_twid_file_stem);
  u_act_wb4_complex_chirp_1024  : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_1024,  c_dm_3, c_pre_ab_1024, c_pre_c_1024, c_un, 0, c_un, 0, c_phasor_chirp_1024,  204800, 51200, FALSE, c_twid_file_stem);
  
  -- * 64 point (as in Apertif channel filterbank)
 u_act_wb1_complex_chirp_64         : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_64,         c_dm_3, c_pre_ab, c_pre_c_64, c_un, 0, c_un, 0, c_phasor_chirp_64,  12800, 12800, FALSE, c_twid_file_stem);
 u_act_wb4_complex_chirp_64         : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_64,         c_dm_3, c_pre_ab, c_pre_c_64, c_un, 0, c_un, 0, c_phasor_chirp_64,  12800, 12800, FALSE, c_twid_file_stem);
 u_act_wb1_complex_flipped_noise_64 : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_flipped_64, c_dm_3, c_pre_ab, c_pre_c_64, c_un, 0, c_un, 0, c_noise_complex_64,  3200,  3200, FALSE, c_twid_file_stem);
 u_act_wb4_complex_flipped_noise_64 : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_flipped_64, c_dm_3, c_pre_ab, c_pre_c_64, c_un, 0, c_un, 0, c_noise_complex_64,  3200,  3200, FALSE, c_twid_file_stem);
  
  -- * 32 point
 u_act_wb4_complex_chirp            : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex,                  c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor_chirp,  6400, 6400, FALSE, c_twid_file_stem);
 u_act_wb4_complex_flipped          : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_flipped,          c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor_chirp,  6400, 6400, FALSE, c_twid_file_stem);
 u_rnd_wb4_complex_flipped_channels : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_flipped_channels, c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor_chirp,  6400, 6400, FALSE, c_twid_file_stem);
 u_rnd_wb1_complex_phasor           : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex,                  c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor,        1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb4_complex_phasor           : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex,                  c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor,        1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb1_complex_fft_shift_phasor : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_fft_shift,        c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor,        1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb4_complex_fft_shift_phasor : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_fft_shift,        c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_phasor,        1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb1_complex_noise            : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex,                  c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb1_complex_noise_channels   : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_4channels,        c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb1_complex_noise_streams    : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb1_complex_4streams,         c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb4_complex_noise            : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex,                  c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb4_complex_noise_channels   : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_4channels,        c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
 u_rnd_wb4_complex_noise_streams    : ENTITY work.tb_wbpfb_unit_wide GENERIC MAP (c_wb4_complex_4streams,         c_dm_3, c_pre_ab, c_pre_c, c_un, 0, c_un, 0, c_noise_complex, 1600, 1600, TRUE, c_twid_file_stem);
END tb;
