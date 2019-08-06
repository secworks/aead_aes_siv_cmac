//======================================================================
//
// siv_cmac_core.v
// ---------------
// Implementation av aead_aes_siv_cmac.
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

                     input wire            init,
                     input wire            next,
                     input wire            finalize,

                     input wire            encdec,
                     input wire [511 : 0]  key,
                     input wire            keylen,

                     input wire [127 : 0]  block,

                     output wire           ready,
                     output wire [127 : 0] result,
                     output wire [127 : 0] tag
                    );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam CTRL_IDLE   = 2'h0;
  localparam CTRL_NEXT   = 2'h1;
  localparam CTRL_LOOP   = 2'h2;
  localparam CTRL_FINISH = 2'h3;


  localparam AES_CMAC = 2'h0;
  localparam AES_CTR  = 2'h1;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          ready_reg;
  reg          ready_new;
  reg          ready_we;

  reg [1 : 0]  core_ctrl_reg;
  reg [1 : 0]  core_ctrl_new;
  reg          core_ctrl_we;


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

  reg           ctr_aes_encdec;
  reg           ctr_aes_init;
  reg           ctr_aes_next;
  reg [255 : 0] ctr_aes_key;
  reg           ctr_aes_keylen;
  reg [127 : 0] ctr_aes_block;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready  = ready_reg;
  assign result = 128'h0;
  assign tag    = 128'h0;


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
          ready_reg     <= 1'h1;
          core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (ready_we)
            ready_reg <= ready_new;

          if (core_ctrl_we)
            core_ctrl_reg <= core_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // siv_cmac_dp
  //
  // The main datapath. Includes the AES access mux.
  //----------------------------------------------------------------
  always @*
    begin : siv_cmac_dp
      aes_mux_ctrl = AES_CMAC;

    end


  //----------------------------------------------------------------
  // AES access mux
  //----------------------------------------------------------------
  always @*
    begin : aes_mux
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
            aes_encdec = ctr_aes_encdec;
            aes_init   = ctr_aes_init;
            aes_next   = ctr_aes_next;
            aes_key    = ctr_aes_key;
            aes_keylen = ctr_aes_keylen;
            aes_block  = ctr_aes_block;
          end

        default:
          begin
            aes_encdec = 1'h0;
            aes_init   = 1'h0;
            aes_next   = 1'h0;
            aes_key    = 256'h0;
            aes_keylen = 1'h0;
            aes_block  = 128'h0;
          end
      endcase // case (aes_mux_ctrl)
    end


  //----------------------------------------------------------------
  // core_ctrl
  //----------------------------------------------------------------
  always @*
    begin : core_ctrl
      ready_new     = 1'h0;
      ready_we      = 1'h0;
      core_ctrl_new = CTRL_IDLE;
      core_ctrl_we  = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (init)
              begin
              end

            if (next)
              begin
              end


            if (finalize)
              begin
              end
          end

        default:
          begin

          end
      endcase // case (core_ctrl_reg)

    end // core_ctrl
endmodule // siv_cmac_core

//======================================================================
// EOF siv_cmac_core.v
//======================================================================
