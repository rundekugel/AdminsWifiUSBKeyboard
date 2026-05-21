#!/usr/bin/env python3
"""
Encrypt an AdminsWifiUSBKeyboard firmware .bin for OTA upload.

Output file format:
  SHA-256(IV || ciphertext)  [32 bytes]  — integrity hash
  IV                         [16 bytes]  — random AES-CBC initialisation vector
  AES-256-CBC(firmware + PKCS#7 padding) [N*16 bytes]

Usage:
  python3 encrypt_firmware.py [options] <input.bin> [output.bin.enc]

  If output path is omitted, <input>.enc is used.

Key selection (first match wins):
  -k <base64>   AES-256 key as a Base64-encoded string (44 chars for 32 bytes)
  -f <file>     AES-256 key read from a 32-byte binary file
  (default)     Hardcoded KEY constant below

Requirements:
  pip install pycryptodome
"""

import sys
import os
import hashlib
import base64
import argparse
from Crypto.Cipher import AES

# -----------------------------------------------------------------------
# Fallback KEY — must match OTA_AES_KEY in main/credentials.h
# To generate a new key:
#   python3 -c "import os; b=os.urandom(32); print(base64.b64encode(b).decode())"
#   python3 -c "import os; b=os.urandom(32); print(', '.join(f'0x{x:02x}' for x in b))"
# -----------------------------------------------------------------------
KEY = bytes([
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
])
# -----------------------------------------------------------------------


def load_key_b64(b64_str: str) -> bytes:
    try:
        key = base64.b64decode(b64_str)
    except Exception as e:
        print(f"Error: invalid Base64 key: {e}", file=sys.stderr)
        sys.exit(1)
    if len(key) != 32:
        print(f"Error: Base64 key must decode to exactly 32 bytes (got {len(key)})", file=sys.stderr)
        sys.exit(1)
    return key


def load_key_hex(hex_str: str) -> bytes:
    # Strip spaces, commas, dots, 0x/0X prefixes, and newlines so the raw
    # credentials.h value can be pasted in directly, e.g.:
    #   "0x00, 0x01, 0x02, ..."  or  "000102..."
    cleaned = hex_str.replace(",", "").replace(".", "").replace(" ", "") \
                     .replace("\t", "").replace("\n", "").replace("\r", "") \
                     .replace("0x", "").replace("0X", "")
    try:
        key = bytes.fromhex(cleaned)
    except ValueError as e:
        print(f"Error: invalid hex key: {e}", file=sys.stderr)
        sys.exit(1)
    if len(key) != 32:
        print(f"Error: hex key must represent exactly 32 bytes (got {len(key)})", file=sys.stderr)
        sys.exit(1)
    return key


def load_key_file(path: str) -> bytes:
    if not os.path.isfile(path):
        print(f"Error: key file not found: {path}", file=sys.stderr)
        sys.exit(1)
    key = open(path, "rb").read()
    if len(key) != 32:
        print(f"Error: key file must be exactly 32 bytes (got {len(key)})", file=sys.stderr)
        sys.exit(1)
    return key


def encrypt(in_path: str, out_path: str, key: bytes) -> None:
    data = open(in_path, "rb").read()

    # PKCS#7 padding to next 16-byte boundary
    pad = 16 - len(data) % 16
    padded = data + bytes([pad] * pad)

    # Random IV for each encryption
    iv = os.urandom(16)

    # AES-256-CBC encrypt
    cipher = AES.new(key, AES.MODE_CBC, iv)
    ciphertext = cipher.encrypt(padded)

    # Encrypt-then-MAC: SHA-256 over IV || ciphertext
    digest = hashlib.sha256(iv + ciphertext).digest()

    with open(out_path, "wb") as f:
        f.write(digest + iv + ciphertext)

    in_kb  = len(data) / 1024
    out_kb = os.path.getsize(out_path) / 1024
    print(f"Input:  {in_path}  ({in_kb:.1f} KB)")
    print(f"Output: {out_path}  ({out_kb:.1f} KB)")
    print(f"SHA-256 of plaintext: {hashlib.sha256(data).hexdigest()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Encrypt firmware .bin for AdminsWifiUSBKeyboard OTA upload.",
        add_help=True,
    )
    parser.add_argument("input",  help="Input firmware .bin file")
    parser.add_argument("output", nargs="?", help="Output encrypted file (default: <input>.enc)")
    key_grp = parser.add_mutually_exclusive_group()
    key_grp.add_argument("-k", metavar="BASE64", help="AES-256 key as Base64 string (32 bytes → 44 chars)")
    key_grp.add_argument("-f", metavar="KEYFILE", help="AES-256 key as 32-byte binary file")
    key_grp.add_argument("-x", metavar="HEX",    help="AES-256 key as hex string; commas, dots, spaces and 0x prefixes are ignored (paste credentials.h value directly)")
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    if args.k:
        key = load_key_b64(args.k)
        print("Key source: -k (Base64)")
    elif args.f:
        key = load_key_file(args.f)
        print(f"Key source: -f {args.f}")
    elif args.x:
        key = load_key_hex(args.x)
        print("Key source: -x (hex)")
    else:
        key = KEY
        print("Key source: hardcoded default")

    out_path = args.output if args.output else args.input + ".enc"
    encrypt(args.input, out_path, key)
