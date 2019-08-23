//======================================================================
//
// tb_core_mem.v
// -------------
// Memory for testing of the aead_aes_siv_cmac core. The memory sports
// variable number of wait states and debug output.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Assured AB
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

module tb_core_mem(
                   input wire            clk,
                   input wire            reset_n,

                   input wire            cs,
                   input wire            we,
                   output wire           ack,
                   input wire [15 : 0]   addr,
                   input wire [127 : 0]  block_wr,
                   output wire [127 : 0] block_rd
                  );


  localparam NUM_WORDS   = 128;
  localparam WAIT_CYCLES = 8'h04;


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  reg [127 : 0] mem [0 : (NUM_WORDS - 1)];
  reg           mem_we;

  reg [7 : 0]   wait_ctr_reg;
  reg [7 : 0]   wait_ctr_new;

  reg           tmp_ack;
  reg [127 : 0] tmp_block_rd;

  reg           ack_reg;
  reg           ack_new;


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  assign ack = ack_reg;
  assign block_rd = tmp_block_rd;


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      integer i;
      if (!reset_n)
        begin
          for (i = 0 ; i < NUM_WORDS ; i = i + 1)
            mem[i] <= 128'h0;

          wait_ctr_reg <= 8'h0;
          ack_reg      <= 1'h0;
        end
      else
        begin
          wait_ctr_reg <= wait_ctr_new;
          ack_reg     <= ack_new;

          if (mem_we)
            mem[addr] = block_wr;
        end
    end // reg_update


  //----------------------------------------------------------------
  // mem_access
  //----------------------------------------------------------------
  always @*
    begin : mem_access;
      mem_we       = 1'h0;
      wait_ctr_new = 8'h0;
      ack_new      = 1'h0;
      tmp_block_rd = 128'h0;

      if (cs)
        begin
          wait_ctr_new = wait_ctr_reg + 1'h1;

          if (wait_ctr_reg >= WAIT_CYCLES)
            begin
              ack_new = 1'h1;
              if (we)
                mem_we = 1'h1;
              else
                tmp_block_rd = mem[addr];
            end
        end
    end
endmodule // tb_core_mem

//======================================================================
// EOF tb_core_mem.v
//======================================================================
