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

  localparam AES_BLOCK_SIZE = 128;


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
      cycle_ctr = cycle_ctr + 1;

      #(CLK_PERIOD);

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
      $display("");

      $display("Internal states:");
      $display("");
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
      cycle_ctr     = 0;
      error_ctr     = 0;
      tc_ctr        = 0;
      debug_ctrl    = 0;

      tb_clk        = 1'h0;
      tb_reset_n    = 1'h1;
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

      display_test_results();

      $display("*** AES_SIV_CORE simulation done. ***");
      $finish;
    end // main

endmodule // tb_aes_siv_core

//======================================================================
// EOF tb_tb_aes_siv_core.v
//======================================================================
