//======================================================================
//
// tb_aes_siv_core.v
// -----------------
// Testbench for the aes_siv_core.
// Testvectors from RFC 5297:
// https://tools.ietf.org/html/rfc5297
//
// Debugged using the aes-siv model by Daniel F Franke:
// https://github.com/dfoxfranke/libaes_siv
//
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Secworks Sweden AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module tb_aes_siv_core();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam DEBUG     = 1;
  localparam DEBUG_MEM = 1;

  localparam CLK_HALF_PERIOD = 1;
  localparam CLK_PERIOD      = 2 * CLK_HALF_PERIOD;

  localparam AEAD_AES_SIV_CMAC_256 = 1'h0;
  localparam AEAD_AES_SIV_CMAC_512 = 1'h1;

  localparam AES_BLOCK_SIZE = 128;

  localparam TIMEOUT_CYCLES = 10000;

  reg [127 : 0] testreg;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0]  cycle_ctr;
  reg [31 : 0]  error_ctr;
  reg [31 : 0]  tc_ctr;
  reg           tc_correct;
  reg           debug_dut;
  reg           debug_mem;

  reg            tb_clk;
  reg            tb_reset_n;
  reg            dut_encdec;
  reg [511 : 0]  dut_key;
  reg            dut_mode;
  reg            dut_start;
  reg [15 :0]    dut_ad_start;
  reg [15 :0]    dut_ad_blocks;
  reg [7 : 0]    dut_ad_final_size;
  reg [15 :0]    dut_pc_start;
  reg [15 :0]    dut_pc_blocks;
  reg [7 : 0]    dut_pc_final_size;
  wire           dut_cs;
  wire           dut_we;
  reg            dut_ack;
  wire [15 : 0]  dut_addr;
  reg [127 : 0]  dut_block_rd;
  wire [127 : 0] dut_block_wr;
  wire [127 : 0] dut_tag_in;
  wire [127 : 0] dut_tag_out;
  wire           dut_tag_ok;
  wire           dut_ready;

  wire           mem_ack;
  reg            mem_cs;
  reg            mem_we;
  reg [15 : 0]   mem_addr;
  reg [127 : 0]  mem_block_wr;
  wire [127 : 0] mem_block_rd;

  reg            tb_debug;
  reg            tb_debug_mem;
  wire           tb_ack;
  reg            tb_cs;
  reg            tb_we;
  reg [15 : 0]   tb_addr;
  reg [7 : 0]    tb_wait_cycles;
  reg [127 : 0]  tb_block_wr;
  reg            tb_mem_ctrl;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  aes_siv_core dut(
                   .clk(tb_clk),
                   .reset_n(tb_reset_n),

                   .encdec(dut_encdec),
                   .key(dut_key),
                   .mode(dut_mode),
                   .start(dut_start),

                   .ad_start(dut_ad_start),
                   .ad_blocks(dut_ad_blocks),
                   .ad_final_size(dut_ad_final_size),

                   .pc_start(dut_pc_start),
                   .pc_blocks(dut_pc_blocks),
                   .pc_final_size(dut_pc_final_size),

                   .cs(dut_cs),
                   .we(dut_we),
                   .ack(dut_ack),
                   .addr(dut_addr),
                   .block_rd(dut_block_rd),
                   .block_wr(dut_block_wr),

                   .tag_in(dut_tag_in),
                   .tag_out(dut_tag_out),
                   .tag_ok(dut_tag_ok),
                   .ready(dut_ready)
                  );


  // Support memory.
  tb_core_mem mem(
                  .clk(tb_clk),
                  .reset_n(tb_reset_n),

                  .wait_cycles(tb_wait_cycles),
                  .debug(tb_debug),
                  .cs(mem_cs),
                  .we(mem_we),
                  .ack(mem_ack),
                  .addr(mem_addr),
                  .block_wr(mem_block_wr),
                  .block_rd(mem_block_rd)
                  );


  //----------------------------------------------------------------
  // mem_access_mux
  // To allow mem access between dut and testbench.
  //----------------------------------------------------------------
  always @*
    begin : mem_access_mux
      if (tb_mem_ctrl)
        begin
          mem_cs       = tb_cs;
          mem_we       = tb_we;
          mem_addr     = tb_addr;
          mem_block_wr = tb_block_wr;
          dut_ack      = 1'h0;
          dut_block_rd = 128'h0;
        end
      else
        begin
          mem_cs       = dut_cs;
          mem_we       = dut_we;
          mem_addr     = dut_addr;
          mem_block_wr = dut_block_wr;
          dut_ack      = mem_ack;
          dut_block_rd = mem_block_rd;
        end
    end


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      #(CLK_PERIOD);
      cycle_ctr = cycle_ctr + 1;

      if (cycle_ctr == TIMEOUT_CYCLES)
        begin
          $display("Timout reached after %d cycles before simulation ended.",
                   cycle_ctr);
          $stop;
        end

      if (debug_dut)
        dump_dut_state();

      if (debug_mem)
        dump_mem_state();
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("cycle:  0x%016x", cycle_ctr);
      $display("Inputs and outputs:");
      $display("ready: 0x%02x, tag: 0x%016x",
               dut.ready, dut.tag_out);
      $display("");

      $display("AES:");
      $display("aes_ready: 0x%01x, aes_init: 0x%01x, aes_next = 0x%01x",
               dut.aes_ready, dut.aes_init, dut.aes_next);
      $display("aes_keylen: 0x%01x, aes_key: 0x%032x", dut.aes_keylen, dut.aes_key);
      $display("aes_block: 0x%016x, aes_result: 0x%016x", dut.aes_block, dut.aes_result);
      $display("");

      $display("CMAC:");
      $display("cmac_ready: 0x%01x, cmac_init: 0x%01x, cmac_next: 0x%01x, cmac_finalize = 0x%01x,  cmac_final_length = 0x%02x",
               dut.cmac_ready, dut.cmac_init, dut.cmac_next, dut.cmac_finalize, dut.cmac_final_size);
      $display("cmac_keylen: 0x%01x, cmac_key: 0x%032x", dut.cmac_keylen, dut.cmac_key);
      $display("cmac_block: 0x%016x, cmac_result: 0x%016x", dut.cmac_block, dut.cmac_result);
      $display("");

      $display("Control and internal states:");
      $display("ctrl_reg: 0x%02x, ctrl_new: 0x%02x, ctrl_we: 0x%01x",
               dut.core_ctrl_reg, dut.core_ctrl_new, dut.core_ctrl_we);
      $display("cmac_inputs: 0x%02x", dut.cmac_inputs);
      $display("s2v_state_reg: 0x%01x, s2v_state_new: 0x%01x, s2v_state_we: 0x%01x",
               dut.s2v_state_reg, dut.s2v_state_new, dut.s2v_state_we);
      $display("d_reg: 0x%016x, d_new: 0x%016x, d_we: 0x%01x",
               dut.d_reg, dut.d_new, dut.d_we);
      $display("v_reg: 0x%016x, v_we: 0x%01x",
               dut.v_reg, dut.v_we);
      $display("x_reg: 0x%016x, x_new: 0x%016x, x_we: 0x%01x",
               dut.x_reg, dut.x_new, dut.x_we);
      $display("\n");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // dump_mem_state()
  //
  // Dump the state of the memory when needed.
  //----------------------------------------------------------------
  task dump_mem_state;
    begin
      $display("cycle: 0x%016x", cycle_ctr);
      $display("Acess mux ctrl: 0x%01x:", tb_mem_ctrl);
      $display("Inputs and outputs:");
      $display("cs: 0x%01x, we: 0x%01x, addr: 0x%04x, ack: 0x%01x",
               mem.cs, mem.we, mem.addr, mem.ack);
      $display("block_wr: 0x%032x", mem.block_wr);
      $display("block_rd: 0x%032x", mem.block_rd);
      $display("");
      $display("Internal states:");
      $display("wait_ctr_reg: 0x%02x, wait_ctr_new: 0x%02x",
               mem.wait_ctr_reg, mem.wait_ctr_new);
      $display("");
      $display("\n");
    end
  endtask // dump_mem_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      $display("TB: Resetting dut.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // display_test_results()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_results;
    begin
      $display("");
      if (error_ctr == 0)
        begin
          $display("%02d test completed. All test cases completed successfully.", tc_ctr);
        end
      else
        begin
          $display("%02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_results


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      debug_dut = 0;

      tb_clk            = 1'h0;
      tb_reset_n        = 1'h1;
      dut_encdec        = 1'h0;
      dut_key           = 512'h0;
      dut_mode          = 1'h0;
      dut_start         = 1'h0;
      dut_ad_start      = 16'h0;
      dut_ad_blocks     = 16'h0;
      dut_ad_final_size = 8'h0;
      dut_pc_start      = 16'h0;
      dut_pc_blocks     = 16'h0;
      dut_pc_final_size = 8'h0;

      tb_debug          = 1'h1;
      tb_wait_cycles    = 8'h2;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // inc_tc_ctr
  //----------------------------------------------------------------
  task inc_tc_ctr;
    tc_ctr = tc_ctr + 1;
  endtask // inc_tc_ctr


  //----------------------------------------------------------------
  // inc_error_ctr
  //----------------------------------------------------------------
  task inc_error_ctr;
    error_ctr = error_ctr + 1;
  endtask // inc_error_ctr


  //----------------------------------------------------------------
  // pause_finish()
  //
  // Pause for a given number of cycles and then finish sim.
  //----------------------------------------------------------------
  task pause_finish(input [31 : 0] num_cycles);
    begin
      $display("Pausing for %04d cycles and then finishing hard.", num_cycles);
      #(num_cycles * CLK_PERIOD);
      $finish;
    end
  endtask // pause_finish


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag to be set in dut.
  //----------------------------------------------------------------
  task wait_ready;
    begin : wready
      while (dut_ready == 0)
        #(CLK_PERIOD);
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // tc1_reset_state
  //
  // Check that registers in the dut are being correctly reset.
  //----------------------------------------------------------------
  task tc1_reset_state;
    begin : tc1
      inc_tc_ctr();
      debug_dut = 1;
      $display("TC1: Check that the dut registers are correctly reset.");
      #(2 * CLK_PERIOD);
      reset_dut();
      #(2 * CLK_PERIOD);
    end
  endtask // tc1_reset_state


  //----------------------------------------------------------------
  // tc2_s2v_init
  //
  // Check that pulling s2v_init perform cmac operation in all
  // zero data and sets the d_reg correctly. Key from RFC 5297.
  //----------------------------------------------------------------
  task tc2_s2v_init;
    begin : tc2
      inc_tc_ctr();
      tc_correct = 1;

      debug_dut = 1;

      $display("TC2: Check that s2v_init works as expected.");
      dut_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      dut_mode = AEAD_AES_SIV_CMAC_256;

      #(2 * CLK_PERIOD);
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_dut = 0;

      if (dut.d_reg != 128'h0e04dfafc1efbf040140582859bf073a)
        begin
          $display("TC2: ERROR - d_reg incorrect. Expected 0x0e04dfafc1efbf040140582859bf073a, got 0x%032x.", dut.d_reg);
          tc_correct = 0;
          inc_error_ctr();
        end

      if (tc_correct)
        $display("TC2: SUCCESS - d_reg correctly initialized.");
      else
        $display("TC2: NO SUCCESS - d_reg not correctly initialized.");
      $display("");
    end
  endtask // tc2


  //----------------------------------------------------------------
  // tc3_s2v_finalize_no_ad
  //
  // Check that pulling s2v_finalize before no AD has been
  // processed leads to v_reg getting the CMAC for all one data.
  // Key from RFC 5297.
  //----------------------------------------------------------------
  task tc3_s2v_finalize_no_ad;
    begin : tc2
      inc_tc_ctr();
      tc_correct = 1;

      debug_dut = 1;

      $display("TC3: Check that v_reg is set when no AD has been processed.");

      $display("TC3: Resetting DUT first.");
      reset_dut();
      #(2 * CLK_PERIOD);

      $display("TC3: Calling s2v finalize.");
      dut_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      dut_mode = AEAD_AES_SIV_CMAC_256;

      #(2 * CLK_PERIOD);
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_dut = 0;

      if (dut.v_reg != 128'h949f99cbcc3eb5da6d3c45d0f59aa9c7)
        begin
          $display("TC2: ERROR - v_reg incorrect. Expected 0x949f99cbcc3eb5da6d3c45d0f59aa9c7, got 0x%032x.", dut.v_reg);
          tc_correct = 0;
          inc_error_ctr();
        end

      if (tc_correct)
        $display("TCC: SUCCESS - v_reg correctly set.");
      else
        $display("TCC: NO SUCCESS - v_reg not correctly set.");
      $display("");
    end
  endtask // tc3_s2v_finalize_no_ad


  //----------------------------------------------------------------
  // tc3_s2v_ad1
  //
  // Check that pulling s2v_finalize before no AD has been
  // processed leads to v_reg getting the CMAC for all one data.
  // Key from RFC 5297.
  //----------------------------------------------------------------
  task tc4_s2v_ad1;
    begin : tc2
      inc_tc_ctr();
      tc_correct = 1;

      debug_dut = 1;

      $display("TC3: Check that v_reg is set when no AD has been processed.");

      $display("TC3: Resetting DUT first.");
      reset_dut();
      #(2 * CLK_PERIOD);

      $display("TC3: Calling s2v finalize.");
      dut_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      dut_mode = AEAD_AES_SIV_CMAC_256;

      #(2 * CLK_PERIOD);
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_dut = 0;

      if (dut.v_reg != 128'h949f99cbcc3eb5da6d3c45d0f59aa9c7)
        begin
          $display("TC2: ERROR - v_reg incorrect. Expected 0x949f99cbcc3eb5da6d3c45d0f59aa9c7, got 0x%032x.", dut.v_reg);
          tc_correct = 0;
          inc_error_ctr();
        end

      if (tc_correct)
        $display("TCC: SUCCESS - v_reg correctly set.");
      else
        $display("TCC: NO SUCCESS - v_reg not correctly set.");
      $display("");
    end
  endtask // tc4_s2v_ad1


  //----------------------------------------------------------------
  // access_test_mem
  //
  // Check that we can read and write to the test memory.
  //----------------------------------------------------------------
  task access_test_mem;
    begin : access_test_mem
      inc_tc_ctr();
      tc_correct = 1;

      debug_mem = 1;

      $display("TCX: Check that we can access the test memory from the TB..");

      // Set access mux to TB. Write data to address 0x0040.
      tb_mem_ctrl = 1'h1;
      tb_addr     = 16'h0040;
      tb_block_wr = 128'hdeadbeef_beefbeef_aaaaaaaa_55555555;
      tb_cs       = 1'h1;
      tb_we       = 1'h1;

      #(10 * CLK_PERIOD);

      tb_cs = 1'h0;
      tb_we = 1'h0;

      $display("TCX: Write to memory should have happened.");
      $display("Contents at address 0x003f: 0x%032x", mem.mem[16'h003f]);
      $display("Contents at address 0x0040: 0x%032x", mem.mem[16'h0040]);
      $display("Contents at address 0x0041: 0x%032x", mem.mem[16'h0041]);

      #(2 * CLK_PERIOD);
      debug_mem = 0;
    end
  endtask // tc4_s2v_ad1


  //----------------------------------------------------------------
  // main
  //
  // The main test functionality.
  //----------------------------------------------------------------
  initial
    begin : main
      $display("*** Testbench for AES_SIV_CORE started ***");
      $display("");

      init_sim();
      reset_dut();
      access_test_mem();

//      tc1_reset_state();
//      tc2_s2v_init();
//      tc3_s2v_finalize_no_ad();

      display_test_results();

      $display("*** AES_SIV_CORE simulation done. ***");
      $finish;
    end // main

endmodule // tb_aes_siv_core

//======================================================================
// EOF tb_tb_aes_siv_core.v
//======================================================================
