//======================================================================
//
// aes_siv_core.v
// --------------
// Implementation av aead_aes_siv_cmac as specified in RFC 5297:
// https://tools.ietf.org/html/rfc5297
//
// The core supports:
// AEAD_AES_SIV_CMAC_256
// AEAD_AES_SIV_CMAC_512
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

module aes_siv_core(
                    input wire            clk,
                    input wire            reset_n,

                    input wire            s2v_init,
                    input wire            s2v_first_block,
                    input wire            s2v_next_block,
                    input wire            s2v_final_block,
                    input wire            s2v_finalize,

                    input wire            ctr_init,
                    input wire            ctr_next,
                    input wire            ctr_finalize,

                    input wire            encdec,
                    input wire [511 : 0]  key,
                    input wire            mode,

                    input wire [127 : 0]  block,
                    input wire [7 : 0]    blocklen,

                    output wire           ready,
                    output wire [127 : 0] result,
                    output wire [127 : 0] tag
                   );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam CTRL_IDLE          = 5'h00;
  localparam CTRL_S2V_INIT0     = 5'h01;
  localparam CTRL_S2V_INIT1     = 5'h02;
  localparam CTRL_S2V_FIRST0    = 5'h03;
  localparam CTRL_S2V_FIRST1    = 5'h04;
  localparam CTRL_S2V_NEXT0     = 5'h05;
  localparam CTRL_S2V_NEXT1     = 5'h06;
  localparam CTRL_S2V_FINAL0    = 5'h07;
  localparam CTRL_S2V_FINAL1    = 5'h08;
  localparam CTRL_S2V_FINALIZE0 = 5'h09;
  localparam CTRL_S2V_FINALIZE1 = 5'h0a;
  localparam CTRL_S2V_FINALIZE2 = 5'h0b;
  localparam CTRL_S2V_FINALIZE3 = 5'h0c;
  localparam CTRL_CTR_INIT0     = 5'h10;
  localparam CTRL_CTR_INIT1     = 5'h11;
  localparam CTRL_CTR_NEXT0     = 5'h12;
  localparam CTRL_CTR_NEXT1     = 5'h13;

  localparam AEAD_AES_SIV_CMAC_256 = 1'h0;
  localparam AEAD_AES_SIV_CMAC_512 = 1'h1;

  localparam CMAC_ZEROES = 2'h0;
  localparam CMAC_ONE    = 2'h1;
  localparam CMAC_BLOCK  = 2'h2;
  localparam CMAC_FINAL  = 2'h3;

  localparam D_CMAC = 2'h0;
  localparam D_DBL  = 2'h1;
  localparam D_XOR  = 2'h2;

  localparam AES_BLOCK_SIZE = 128;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg           ready_reg;
  reg           ready_new;
  reg           ready_we;

  reg [127 : 0] block_reg;
  reg           block_we;

  reg [127 : 0] d_reg;
  reg [127 : 0] d_new;
  reg           d_we;

  reg [127 : 0] v_reg;
  reg           v_we;

  reg [127 : 0] x_reg;
  reg [127 : 0] x_new;
  reg           x_we;

  reg           s2v_state_reg;
  reg           s2v_state_new;
  reg           s2v_state_we;

  reg [127 : 0] result_reg;
  reg [127 : 0] result_new;
  reg           result_we;

  reg [3 : 0]   core_ctrl_reg;
  reg [3 : 0]   core_ctrl_new;
  reg           core_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg            aes_encdec;
  reg            aes_init;
  reg            aes_next;
  reg [255 : 0]  aes_key;
  reg            aes_keylen;
  reg [127 : 0]  aes_block;
  wire [127 : 0] aes_result;
  wire           aes_ready;
  wire           aes_valid;

  reg [255 : 0]  cmac_key;
  reg            cmac_keylen;
  reg [7 : 0]    cmac_final_size;
  reg            cmac_init;
  reg            cmac_next;
  reg            cmac_finalize;
  reg [127 : 0]  cmac_block;
  wire [127 : 0] cmac_result;
  wire           cmac_ready;
  wire           cmac_valid;
  reg [1 : 0]    cmac_inputs;

  reg            init_ctr;
  reg            update_ctr;

  reg            update_d;
  reg [1 : 0]    ctrl_d;

  reg            update_v;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready  = ready_reg;
  assign result = result_reg;
  assign tag    = v_reg;


  //----------------------------------------------------------------
  // Functions.
  //----------------------------------------------------------------
  function [127 : 0] double(input [127 : 0] op);
    begin
      if (op[127])
        double = {op[126 : 0], 1'h0} ^ 128'h87;
      else
        double = {op[126 : 0], 1'h0};
    end
  endfunction


  //----------------------------------------------------------------
  // core instantiations.
  //----------------------------------------------------------------
  // AES core is only used for CTR part.
  aes_core aes(
               .clk(clk),
               .reset_n(reset_n),

               .encdec(aes_encdec),
               .init(aes_init),
               .next(aes_next),
               .ready(aes_ready),

               .key(aes_key),
               .keylen(aes_keylen),

               .block(aes_block),
               .result(aes_result),
               .result_valid(aes_valid)
               );


  cmac_core cmac(
                 .clk(clk),
                 .reset_n(reset_n),
                 .key(cmac_key),
                 .keylen(cmac_keylen),
                 .final_size(cmac_final_size),
                 .init(cmac_init),
                 .next(cmac_next),
                 .finalize(cmac_finalize),
                 .block(cmac_block),
                 .result(cmac_result),
                 .ready(cmac_ready),
                 .valid(cmac_valid)
                );


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (!reset_n)
        begin
          ready_reg      <= 1'h1;
          block_reg      <= 128'h0;
          result_reg     <= 128'h0;
          d_reg          <= 128'h0;
          v_reg          <= 128'h0;
          x_reg          <= 128'h0;
          s2v_state_reg  <= 1'h0;
          core_ctrl_reg  <= CTRL_IDLE;
        end
      else
        begin
          if (ready_we)
            ready_reg <= ready_new;

          if (block_we)
            block_reg <= block;

          if (d_we)
            d_reg <= d_new;

          if (v_we)
            v_reg <= cmac_result;

          if (s2v_state_we)
            s2v_state_reg <= s2v_state_new;

          if (x_we)
            x_reg <= x_new;

          if (result_we)
            result_reg <= result_new;

          if (core_ctrl_we)
            core_ctrl_reg <= core_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // s2v_dp
  //
  // Datapath for the S2V functionality.
  // Note that the S2V functionality assumes that the CMAC core
  // has access to the AES core.
  //----------------------------------------------------------------
  always @*
    begin : siv_cmac_dp
      d_new = 128'h0;
      d_we  = 1'h0;
      v_we  = 1'h0;

      cmac_block  = 128'h0;
      cmac_key    = key[511 : 256];
      cmac_keylen = mode;

      case (cmac_inputs)
        CMAC_ZEROES: cmac_block = 128'h0;
        CMAC_ONE:    cmac_block = 128'h1;
        CMAC_BLOCK:  cmac_block = block_reg;
        CMAC_FINAL:  cmac_block = d_reg;
      endcase // case (cmac_inputs)

      if (update_v)
        begin
          v_we  = 1'h1;
        end

      if (update_d)
        begin
          d_we = 1'h1;
          case (ctrl_d)
            D_CMAC: d_new = cmac_result;
            D_DBL:  d_new = double(d_reg);
            D_XOR:  d_new = d_reg ^ cmac_result;
            default
              begin
              end
          endcase // case (d_ctrl)
        end
    end


  //----------------------------------------------------------------
  // ctr_dp
  //
  // Datapath for the CTR functionality.
  // Note that the CTR functionality assumes that it has access
  // to the AES core.
  //----------------------------------------------------------------
  always @*
    begin : ctr_dp
      reg [63 : 0] x_tmp;

      x_new = 128'h0;
      x_we  = 1'h0;

      // Clear bit 63 and 31 when seeding the counter.
      // See RFC 5297, Section 2.5.
      if (init_ctr)
        begin
          x_new = {v_reg[127 : 64], 1'h0, v_reg[62 : 32], 1'h0, v_reg[30 : 0]};
          x_we  = 1'h1;
        end

      // 64 bit adder used.
      if (update_ctr)
        begin
          result_new = block_reg ^ aes_result;
          result_we  = 1'h1;
          x_tmp = x_reg[63 : 0] + 1'h1;
          x_new = {x_reg[127 : 64], x_tmp};
          x_we  = 1'h1;
        end
    end // ctr_dp


  //----------------------------------------------------------------
  // core_ctrl
  //----------------------------------------------------------------
  always @*
    begin : core_ctrl
      ready_new       = 1'h0;
      ready_we        = 1'h0;
      cmac_final_size = 8'h0;
      cmac_init       = 1'h0;
      cmac_next       = 1'h0;
      cmac_finalize   = 1'h0;
      s2v_state_new   = 1'h0;
      s2v_state_we    = 1'h0;
      init_ctr        = 1'h0;
      update_ctr      = 1'h0;
      block_we        = 1'h0;
      result_new      = 128'h0;
      result_we       = 1'h0;
      cmac_inputs     = CMAC_ZEROES;
      update_d        = 1'h0;
      ctrl_d          = D_CMAC;
      update_v        = 1'h0;
      core_ctrl_new   = CTRL_IDLE;
      core_ctrl_we    = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (s2v_init)
              begin
                cmac_init     = 1'h1;
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_INIT0;
                core_ctrl_we  = 1'h1;
              end

            if (s2v_first_block)
              begin
                cmac_init     = 1'h1;
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                block_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_FIRST0;
                core_ctrl_we  = 1'h1;
              end

            if (s2v_next_block)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                block_we      = 1'h1;
                block_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_NEXT0;
                core_ctrl_we  = 1'h1;
              end

            if (s2v_final_block)
              begin
                block_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_FINAL0;
                core_ctrl_we  = 1'h1;
              end

            if (s2v_finalize)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_FINALIZE0;
                core_ctrl_we  = 1'h1;
              end

            if (ctr_init)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_CTR_INIT0;
                core_ctrl_we  = 1'h1;
              end

            if (ctr_next)
              begin
                block_we      = 1'h1;
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_CTR_NEXT0;
                core_ctrl_we  = 1'h1;
              end

            if (ctr_finalize)
              begin
                ready_new = 1'h0;
                ready_we  = 1'h1;
                block_we  = 1'h1;
              end
          end


        CTRL_S2V_INIT0:
          begin
            if (cmac_ready)
              begin
                cmac_inputs     = CMAC_ZEROES;
                cmac_final_size = AES_BLOCK_SIZE;
                cmac_finalize   = 1'h1;
                core_ctrl_new   = CTRL_S2V_INIT1;
                core_ctrl_we    = 1'h1;
              end
          end


        CTRL_S2V_INIT1:
          begin
            cmac_inputs     = CMAC_ZEROES;
            cmac_final_size = AES_BLOCK_SIZE;
            if (cmac_ready)
              begin
                s2v_state_new = 1'h1;
                s2v_state_we  = 1'h1;
                update_d      = 1'h1;
                ctrl_d        = D_CMAC;
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FIRST0:
          begin
            if (cmac_ready)
              begin
                cmac_next     = 1'h1;
                cmac_inputs   = CMAC_BLOCK;
                core_ctrl_new = CTRL_S2V_FIRST1;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FIRST1:
          begin
            cmac_inputs = CMAC_BLOCK;
            if (cmac_ready)
              begin
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_NEXT0:
          begin
            cmac_next     = 1'h1;
            cmac_inputs   = CMAC_BLOCK;
            core_ctrl_new = CTRL_S2V_NEXT1;
            core_ctrl_we  = 1'h1;
          end


        CTRL_S2V_NEXT1:
          begin
            cmac_inputs = CMAC_BLOCK;
            if (cmac_ready)
              begin
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FINAL0:
          begin
            update_d        = 1'h1;
            ctrl_d          = D_DBL;
            cmac_finalize   = 1'h1;
            cmac_inputs     = CMAC_BLOCK;
            cmac_final_size = blocklen;
            core_ctrl_new   = CTRL_S2V_FINAL1;
            core_ctrl_we    = 1'h1;
          end


        CTRL_S2V_FINAL1:
          begin
            cmac_inputs     = CMAC_BLOCK;
            cmac_final_size = blocklen;
            if (cmac_ready)
              begin
                update_d      = 1'h1;
                ctrl_d        = D_XOR;
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FINALIZE0:
          begin
            // Check if cmac and s2v has been initalized.
            if (!s2v_state_reg)
              begin
                cmac_init       = 1'h1;
                core_ctrl_new   = CTRL_S2V_FINALIZE1;
                core_ctrl_we    = 1'h1;
              end
            else
              begin
                core_ctrl_new = CTRL_S2V_FINALIZE3;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FINALIZE1:
          begin
            // Check if cmac and s2v has been initalized.
            if (cmac_ready)
              begin
                cmac_finalize   = 1'h1;
                cmac_inputs     = CMAC_ONE;
                cmac_final_size = AES_BLOCK_SIZE;
                core_ctrl_new   = CTRL_S2V_FINALIZE2;
                core_ctrl_we    = 1'h1;
              end
          end


        // Handle case when no AAD has been processed.
        CTRL_S2V_FINALIZE2:
          begin
            cmac_inputs     = CMAC_ONE;
            cmac_final_size = AES_BLOCK_SIZE;
            if (cmac_ready)
              begin
                update_v      = 1'h1;
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        // Handle finalization when AD has been processed.
        CTRL_S2V_FINALIZE3:
          begin
            ready_new     = 1'h1;
            ready_we      = 1'h1;
            core_ctrl_new = CTRL_IDLE;
            core_ctrl_we  = 1'h1;
          end


        CTRL_CTR_INIT0:
          begin
            init_ctr      = 1'h1;
            core_ctrl_new = CTRL_CTR_INIT1;
            core_ctrl_we  = 1'h1;
          end


        CTRL_CTR_INIT1:
          begin
            if (aes_ready)
              begin
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_CTR_NEXT0:
          begin
            core_ctrl_new = CTRL_CTR_NEXT1;
            core_ctrl_we  = 1'h1;
          end


        CTRL_CTR_NEXT1:
          begin
            if (aes_ready)
              begin
                update_ctr    = 1'h1;
                result_we     = 1'h1;
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        default:
          begin

          end
      endcase // case (core_ctrl_reg)
    end // block: core_ctrl
endmodule // aes_siv_core

//======================================================================
// EOF aes_siv_core.v
//======================================================================
