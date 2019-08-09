//======================================================================
//
// siv_cmac_core.v
// ---------------
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

module siv_cmac_core(
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
  localparam CTRL_IDLE          = 4'h0;
  localparam CTRL_S2V_INIT0     = 4'h1;
  localparam CTRL_S2V_INIT1     = 4'h2;
  localparam CTRL_S2V_FINALIZE0 = 4'h6;
  localparam CTRL_S2V_FINALIZE1 = 4'h7;
  localparam CTRL_CTR_INIT0     = 4'h8;
  localparam CTRL_CTR_INIT1     = 4'h9;
  localparam CTRL_CTR_NEXT0     = 4'ha;
  localparam CTRL_CTR_NEXT1     = 4'hb;
  localparam CTRL_WAIT_READY    = 4'hf;

  localparam AES_CMAC = 2'h0;
  localparam AES_CTR  = 2'h1;

  localparam AEAD_AES_SIV_CMAC_256 = 1'h0;
  localparam AEAD_AES_SIV_CMAC_512 = 1'h1;

  localparam CMAC_ZEROES = 2'h0;
  localparam CMAC_ONES   = 2'h1;
  localparam CMAC_DATA   = 2'h2;
  localparam CMAC_FINAL  = 2'h3;

  localparam D_INIT = 2'h0;
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
  reg [1 : 0]    aes_mux_ctrl;
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
  wire           cmac_aes_encdec;
  wire           cmac_aes_init;
  wire           cmac_aes_next;
  wire [255 : 0] cmac_aes_key;
  wire           cmac_aes_keylen;
  wire [127 : 0] cmac_aes_block;
  wire [127 : 0] cmac_result;
  wire           cmac_ready;
  wire           cmac_valid;

  reg            init_ctr;
  reg            update_ctr;

  reg            ctr_aes_init;
  reg            ctr_aes_next;
  reg [127 : 0]  ctr_aes_block;

  reg [1 : 0]    cmac_inputs;

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
                 .aes_encdec(cmac_aes_encdec),
                 .aes_init(cmac_aes_init),
                 .aes_next(cmac_aes_next),
                 .aes_ready(aes_ready),
                 .aes_key(cmac_aes_key),
                 .aes_keylen(cmac_aes_keylen),
                 .aes_block(cmac_aes_block),
                 .aes_result(aes_result),
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
            v_reg <= aes_result;

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
      v_we            = 1'h0;

      cmac_block      = 128'h0;
      cmac_key        = key[511 : 256];
      cmac_keylen     = mode;

      if (update_v)
        begin
          v_we  = 1'h1;
        end

      if (update_d)
        begin
          d_we = 1'h1;
          case (ctrl_d)
            D_INIT: d_new = cmac_result;
            D_DBL:  d_new = double(d_reg);
            D_XOR:  d_new = d_reg ^ cmac_result;
            default
              begin
              end
          endcase // case (d_ctrl)
        end

      case (cmac_inputs)
        CMAC_ZEROES: cmac_block = 128'h0;
        CMAC_ONES:   cmac_block = {128{1'h1}};
        CMAC_DATA:   cmac_block = block_reg;
        CMAC_FINAL:  cmac_block = d_reg;
      endcase // case (cmac_inputs)
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
  // AES access mux
  //----------------------------------------------------------------
  always @*
    begin : aes_mux
      aes_encdec = 1'h0;
      aes_init   = 1'h0;
      aes_next   = 1'h0;
      aes_key    = 256'h0;
      aes_keylen = 1'h0;
      aes_block  = 128'h0;

      case (aes_mux_ctrl)
        AES_CMAC:
          begin
            aes_encdec = cmac_aes_encdec;
            aes_init   = cmac_aes_init;
            aes_next   = cmac_aes_next;
            aes_key    = cmac_aes_key;
            aes_keylen = cmac_aes_keylen;
            aes_block  = cmac_aes_block;
          end

        AES_CTR:
          begin
            aes_encdec = 1'h1;
            aes_init   = ctr_aes_init;
            aes_next   = ctr_aes_next;
            aes_key    = key[255 : 0];
            aes_keylen = mode;
            aes_block  = x_reg;
          end

        default:
          begin
          end
      endcase // case (aes_mux_ctrl)
    end


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
      ctr_aes_init    = 1'h0;
      ctr_aes_next    = 1'h0;
      block_we        = 1'h0;
      result_new      = 128'h0;
      result_we       = 1'h0;
      aes_mux_ctrl    = AES_CMAC;
      cmac_inputs     = CMAC_ZEROES;
      update_d        = 1'h0;
      ctrl_d          = D_INIT;
      update_v        = 1'h0;
      core_ctrl_new   = CTRL_IDLE;
      core_ctrl_we    = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (s2v_init)
              begin
                cmac_init     = 1'h1;
                aes_mux_ctrl  = AES_CMAC;
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_S2V_INIT0;
                core_ctrl_we  = 1'h1;
              end

            if (s2v_first_block)
              begin
                block_we      = 1'h1;
              end


            if (s2v_next_block)
              begin
                block_we      = 1'h1;
              end

            if (s2v_final_block)
              begin
                block_we      = 1'h1;
              end

            if (s2v_finalize)
              begin
                cmac_init     = 1'h1;
                block_we      = 1'h1;
                aes_mux_ctrl  = AES_CMAC;
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
                block_we = 1'h1;
              end
          end


        CTRL_S2V_INIT0:
          begin
            aes_mux_ctrl  = AES_CMAC;
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
            aes_mux_ctrl  = AES_CMAC;

            if (cmac_ready)
              begin
                s2v_state_new = 1'h1;
                s2v_state_we  = 1'h1;

                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FINALIZE0:
          begin
            aes_mux_ctrl  = AES_CMAC;

            if (cmac_ready)
              begin
                if (!s2v_state_reg)
                  begin
                    cmac_inputs     = CMAC_ONES;
                    cmac_final_size = AES_BLOCK_SIZE;
                  end
                else
                  begin

                  end
                cmac_finalize = 1'h1;
                core_ctrl_new = CTRL_S2V_FINALIZE1;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_S2V_FINALIZE1:
          begin
            if (cmac_ready)
              begin
                s2v_state_new = 1'h0;
                s2v_state_we  = 1'h1;
                ready_new     = 1'h1;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_IDLE;
                core_ctrl_we  = 1'h1;
              end
          end


        CTRL_CTR_INIT0:
          begin
            init_ctr      = 1'h1;
            ctr_aes_init  = 1'h1;
            aes_mux_ctrl  = AES_CTR;
            core_ctrl_new = CTRL_CTR_INIT1;
            core_ctrl_we  = 1'h1;
          end


        CTRL_CTR_INIT1:
          begin
            aes_mux_ctrl = AES_CTR;

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
            ctr_aes_next  = 1'h1;
            aes_mux_ctrl  = AES_CTR;
            core_ctrl_new = CTRL_CTR_NEXT1;
            core_ctrl_we  = 1'h1;
          end


        CTRL_CTR_NEXT1:
          begin
            aes_mux_ctrl = AES_CTR;

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
endmodule // siv_cmac_core

//======================================================================
// EOF siv_cmac_core.v
//======================================================================
