-- A thin interface implementation of the fft_wide_unit.
-- This wrapper removes the wideband_fft_control unit which was 
-- causing timing issues.
-- Author: Talon Myburgh
library ieee, common_pkg_lib, casper_ram_lib, r2sdf_fft_lib, casper_requantize_lib;
use IEEE.std_logic_1164.all;
use common_pkg_lib.common_pkg.all;
use casper_ram_lib.common_ram_pkg.all;
use r2sdf_fft_lib.rTwoSDFPkg.all;
use work.fft_gnrcs_intrfcs_pkg.all;

entity fft_wide_unit_thin is
	generic(
		g_fft          	 	: t_fft          	:= c_fft; 				--! generics for the FFT
		g_pft_pipeline 	 	: t_fft_pipeline 	:= c_fft_pipeline; 		--! For the pipelined part, defined in casper_r2sdf_fft_lib.rTwoSDFPkg
		g_fft_pipeline 	 	: t_fft_pipeline 	:= c_fft_pipeline; 		--! For the parallel part, defined in casper_r2sdf_fft_lib.rTwoSDFPkg
		g_use_variant    	: string  			:= "4DSP";        		--! = "4DSP" or "3DSP" for 3 or 4 mult cmult.
		g_use_dsp        	: string  			:= "yes";        		--! = "yes" or "no"
		g_ovflw_behav    	: string  			:= "WRAP";        		--! = "WRAP" or "SATURATE" will default to WRAP if invalid option used
		g_use_round      	: string  			:= "ROUND";        		--! = "ROUND" or "TRUNCATE" will default to TRUNCATE if invalid option used
		g_ram_primitive  	: string  			:= "auto";        		--! = "auto", "distributed", "block" or "ultra" for RAM architecture
		g_fifo_primitive 	: string  			:= "auto";        		--! = "auto", "distributed", "block" or "ultra" for RAM architecture
		g_twid_file_stem	: string			:= c_twid_file_stem		--! path stem for twiddle factors
	);
	port(
		clken           	: in  std_logic := '1';																--! Clock enable
		rst             	: in  std_logic := '0';																--! Reset
		clk             	: in  std_logic := '1';																--! Clock
		in_valid			: in  std_logic := '1';																--! In valid signal
		shiftreg 		    : in  std_logic_vector(ceil_log2(g_fft.nof_points) - 1 DOWNTO 0) := (others=>'1');	--! Shift register
		in_fft_re_arr       : in  t_fft_slv_arr_in(g_fft.wb_factor -1 downto 0) := (others=>(others=>'0'));		--! Input data array (wb_factor wide)
		in_fft_im_arr       : in  t_fft_slv_arr_in(g_fft.wb_factor -1 downto 0) := (others=>(others=>'0'));		--! Input data array (wb_factor wide)
		out_valid			: out  std_logic;																	--! In valid signal
		ovflw				: out std_logic_vector(ceil_log2(g_fft.nof_points) - 1 DOWNTO 0);					--!	Overflow register
		out_fft_re_arr      : out t_fft_slv_arr_out(g_fft.wb_factor -1 downto 0);								--! Output data array (wb_factor wide)
		out_fft_im_arr      : out t_fft_slv_arr_out(g_fft.wb_factor -1 downto 0)								--! Output data array (wb_factor wide)
	);
end entity fft_wide_unit_thin;

architecture str of fft_wide_unit_thin is
	
	signal fft_in_re_arr : t_fft_slv_arr_in(g_fft.wb_factor - 1 downto 0);
	signal fft_in_im_arr : t_fft_slv_arr_in(g_fft.wb_factor - 1 downto 0);

	signal fft_out_re_arr : t_fft_slv_arr_out(g_fft.wb_factor - 1 downto 0);
	signal fft_out_im_arr : t_fft_slv_arr_out(g_fft.wb_factor - 1 downto 0);
	signal fft_out_val    : std_logic;

	signal fft_shiftreg	   :  std_logic_vector(ceil_log2(g_fft.nof_points) - 1 downto 0);

	type reg_type is record
		in_fft_re_arr :  t_fft_slv_arr_in(g_fft.wb_factor -1 downto 0);
		in_fft_im_arr :  t_fft_slv_arr_in(g_fft.wb_factor -1 downto 0);
		in_valid	  :  std_logic;
		shiftreg	  :  std_logic_vector(ceil_log2(g_fft.nof_points) - 1 downto 0);
	end record;

	signal r, rin : reg_type;

begin
	---------------------------------------------------------------
	-- INPUT REGISTER FOR THE INPUT SIGNALS
	---------------------------------------------------------------
	-- The complete set of input signals are registered.
	
	comb : process(r, in_fft_re_arr)
		variable v : reg_type;
	begin
		v             	:= r;
		v.in_valid		:= in_valid;
		v.in_fft_re_arr := in_fft_re_arr;
		v.in_fft_re_arr := in_fft_re_arr;
		v.shiftreg	  	:= shiftreg;
		rin           	<= v;
	end process comb;

	regs : process(clken, clk)
	begin
		if rising_edge(clk) and clken = '1' then
			r <= rin;
		end if;
	end process;

	---------------------------------------------------------------
	-- PREPARE INPUT DATA FOR WIDEBAND FFT
	---------------------------------------------------------------
	-- Extract the data from the in_fft_sosi_arr records and resize it 
	-- to fit the format of the fft_r2_wide unit. 

	fft_in_re_arr <= r.in_fft_re_arr;
	fft_in_im_arr <= r.in_fft_re_arr;
	fft_shiftreg <= r.shiftreg;

	---------------------------------------------------------------
	-- THE WIDEBAND FFT
	---------------------------------------------------------------
	u_fft_wide : entity work.fft_r2_wide
		generic map(
			g_fft          		=> g_fft,    -- generics for the WFFT
			g_pft_pipeline 		=> g_pft_pipeline,
			g_fft_pipeline 		=> g_fft_pipeline,
			g_use_variant  		=> g_use_variant,
			g_use_dsp	   		=> g_use_dsp,
			g_ovflw_behav		=> g_ovflw_behav,
			g_use_round			=> g_use_round,
			g_ram_primitive		=> g_ram_primitive,
			g_fifo_primitive	=> g_fifo_primitive,
			g_twid_file_stem	=> g_twid_file_stem
		)
		port map(
			clken      			=> clken,
			clk        			=> clk,
			rst        			=> rst,
			shiftreg   			=> fft_shiftreg,
			in_re_arr  			=> fft_in_re_arr,
			in_im_arr  			=> fft_in_im_arr,
			in_val     			=> r.in_valid,
			out_re_arr 			=> fft_out_re_arr,
			out_im_arr 			=> fft_out_im_arr,
			ovflw	   			=> ovflw,
			out_val    			=> out_valid
		);

end str;
