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
  localparam DEBUG = 1;

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
  reg           debug_ctrl;

  reg            tb_clk;
  reg            tb_reset_n;

  reg            tb_s2v_init;
  reg            tb_s2v_first_block;
  reg            tb_s2v_next_block;
  reg            tb_s2v_final_block;
  reg            tb_s2v_finalize;
  reg            tb_ctr_init;
  reg            tb_ctr_next;
  reg            tb_ctr_finalize;
  reg            tb_encdec;
  reg [511 : 0]  tb_key;
  reg            tb_mode;
  reg [127 : 0]  tb_block;
  reg [7 : 0]    tb_blocklen;
  wire           tb_ready;
  wire [127 : 0] tb_result;
  wire [127 : 0] tb_tag;


  //----------------------------------------------------------------
  // Instantiations.
  //----------------------------------------------------------------
  aes_siv_core dut(
                   .clk(tb_clk),
                   .reset_n(tb_reset_n),
                   .s2v_init(tb_s2v_init),
                   .s2v_first_block(tb_s2v_first_block),
                   .s2v_next_block(tb_s2v_next_block),
                   .s2v_final_block(tb_s2v_final_block),
                   .s2v_finalize(tb_s2v_finalize),
                   .ctr_init(tb_ctr_init),
                   .ctr_next(tb_ctr_next),
                   .ctr_finalize(tb_ctr_finalize),
                   .encdec(tb_encdec),
                   .key(tb_key),
                   .mode(tb_mode),
                   .block(tb_block),
                   .blocklen(tb_blocklen),
                   .ready(tb_ready),
                   .result(tb_result),
                   .tag(tb_tag)
                  );


  //----------------------------------------------------------------
  // Concurrent assignments.
  //----------------------------------------------------------------


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

      if (debug_ctrl)
        begin
          dump_dut_state();
        end
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
      $display("s2v_init: 0x%01x, s2v_finalize: 0x%01x", dut.s2v_init, dut.s2v_finalize);
      $display("s2v_first_block: 0x%01x, s2v_next_block: 0x%01x, s2v_final_block: 0x%01x",
               dut.s2v_first_block, dut.s2v_next_block, dut.s2v_final_block);
      $display("ctr_init: 0x%01x, ctr_next: 0x%01x, ctr_finalize: 0x%01x",
               dut.ctr_init, dut.ctr_next, dut.ctr_finalize);
      $display("key: 0x%064x", dut.key);
      $display("blocklen: 0x%02x, block: 0x%016x", dut.blocklen, dut.block);
      $display("ready: 0x%02x, result: 0x%016x, tag: 0x%016x",
               dut.ready, dut.result, dut.tag);
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
      debug_ctrl = 0;

      tb_clk             = 1'h0;
      tb_reset_n         = 1'h1;
      tb_s2v_init        = 1'h0;
      tb_s2v_first_block = 1'h0;
      tb_s2v_next_block  = 1'h0;
      tb_s2v_final_block = 1'h0;
      tb_s2v_finalize    = 1'h0;
      tb_ctr_init        = 1'h0;
      tb_ctr_next        = 1'h0;
      tb_ctr_finalize    = 1'h0;
      tb_encdec          = 1'h0;
      tb_key             = 512'h0;
      tb_mode            = 1'h0;
      tb_block           = 128'h0;
      tb_blocklen        = 1'h0;
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
      while (tb_ready == 0)
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
      debug_ctrl = 1;
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

      debug_ctrl = 1;

      $display("TC2: Check that s2v_init works as expected.");
      tb_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      tb_mode = AEAD_AES_SIV_CMAC_256;

      tb_s2v_init = 1'h1;
      #(2 * CLK_PERIOD);
      tb_s2v_init = 1'h0;
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_ctrl = 0;

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

      debug_ctrl = 1;

      $display("TC3: Check that v_reg is set when no AD has been processed.");

      $display("TC3: Resetting DUT first.");
      reset_dut();
      #(2 * CLK_PERIOD);

      $display("TC3: Calling s2v finalize.");
      tb_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      tb_mode = AEAD_AES_SIV_CMAC_256;

      tb_s2v_finalize = 1'h1;
      #(2 * CLK_PERIOD);
      tb_s2v_finalize = 1'h0;
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_ctrl = 0;

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

      debug_ctrl = 1;

      $display("TC3: Check that v_reg is set when no AD has been processed.");

      $display("TC3: Resetting DUT first.");
      reset_dut();
      #(2 * CLK_PERIOD);

      $display("TC3: Calling s2v finalize.");
      tb_key  = {128'hfffefdfc_fbfaf9f8_f7f6f5f4_f3f2f1f0, {128{1'h0}},
                   128'hf0f1f2f3_f4f5f6f7_f8f9fafb_fcfdfeff, {128{1'h0}}};
      tb_mode = AEAD_AES_SIV_CMAC_256;

      tb_s2v_finalize = 1'h1;
      #(2 * CLK_PERIOD);
      tb_s2v_finalize = 1'h0;
      wait_ready();

      #(2 * CLK_PERIOD);
      debug_ctrl = 0;

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
  // main
  //
  // The main test functionality.
  //----------------------------------------------------------------
  initial
    begin : main
      $display("*** Testbench for AES_SIV_CORE started ***");
      $display("");

      init_sim();

      tc1_reset_state();
      tc2_s2v_init();
      tc3_s2v_finalize_no_ad();

      display_test_results();

      $display("*** AES_SIV_CORE simulation done. ***");
      $finish;
    end // main

endmodule // tb_aes_siv_core

//======================================================================
// EOF tb_tb_aes_siv_core.v
//======================================================================
