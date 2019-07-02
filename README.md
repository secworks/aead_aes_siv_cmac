# aead_aes_siv_cmac
Hardware implementation of AEAD_AES_SIV_CMAC

## Status
Not completed. Does **NOT** work. Do **NOT** use.


## Introduction
The authenticated encryption (AE) block cipher mode AEAD_AES_SIV_CMAC is
block cipher mode using CTR mode (NIST SP 800-38A) for encryption and
CMAC mode (NIST SP 800-38B) for authentication. CMAC is also used to
generate the synthetic IV (nonce) for CTR. The mode also supports
additional (or associated) data (AD) that is authenticated bot not
encrypted.

The AE mode is specified in [RFC 5297 - Synthetic Initialization Vector
(SIV) Authenticated Encryption Using the Advanced Encryption Standard
(AES)](https://tools.ietf.org/html/rfc5297).

The implementation will support AEAD_AES_SIV_CMAC_256 and quite probably
also support AEAD_AES_SIV_CMAC_512. The implementation will not support
AEAD_AES_SIV_CMAC_384 since [the aes
core](https://github.com/secworks/aes) does not support 192 bit keys.


## Implementation
TBW.
