/*
 * Copyright (C) 2015  Mozilla Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>

/*
 * The data type |struct pdu| represents a single PDU as transfered
 * over an I/O channel. A PDU has the following format
 *
 *    |    1    |    1    |        2       |    n    |
 *    | service |  opcode | payload length | payload |
 *
 * Each PDU consists of a 4-byte header with service, opcode, and
 * payload length; and a variable-length data buffer. The service
 * describes the module and opcode describes the operation of the
 * service. The payload length contains the number of valid data
 * bytes in the payload.
 *
 * The PDU is based on the format of the BlueZ HAL protocol, which
 * is defined at
 *
 *    https://git.kernel.org/cgit/bluetooth/bluez.git/tree/android/hal-ipc-api.txt
 *
 * A PDU can be initialized with a call to |init_pdu|. Payload is
 * read and written with |read_pdu_at|, |write_pdu_at|, and
 * |append_to_pdu|. The latter will automatically update the length
 * field in the PDU header.
 *
 * Similar to |scanf| and |printf|, the types of the marshalled data
 * is described by format-string characters. The types are
 *
 *      Character |   Type
 *    ------------+-----------------
 *        'c'     | signed 8 bit
 *        'C'     | unsigned 8 bit
 *        's'     | signed 16 bit
 *        'S'     | unsigned 16 bit
 *        'i'     | signed 32 bit
 *        'I'     | unsigned 32 bit
 *        'l'     | signed 64 bit
 *        'L'     | unsigned 64 bit
 *        'f'     | 32-bit floating point
 *        'd'     | 64-bit floating point
 *        'm'     | raw memory
 *        'M'     | raw memory
 *        '0'     | raw 0-terminated memory
 *
 * The characters 'm' and 'M' require pointers to the raw memory and the
 * length of the memory block in bytes given as a |size_t|. Capital |M|
 * will allocate the required memory automatically when reading from the
 * PDU. The caller is responsible for freeing the memory with |free|.
 */

#define PDU_BYTE_WIDTH (8) /* bits per byte in a PDU */

#define PDU_HEADER_SERVICE_SIZE (1) /* size of the service field in bytes */
#define PDU_HEADER_OPCODE_SIZE  (1) /* size of the opcode field in bytes */
#define PDU_HEADER_LENGTH_SIZE  (2) /* size of the length field in bytes */

/* PDU header size in bytes */
#define PDU_HEADER_SIZE \
  (PDU_HEADER_SERVICE_SIZE + PDU_HEADER_OPCODE_SIZE + PDU_HEADER_LENGTH_SIZE)

/* maximum PDU data size in bytes */
#define PDU_MAX_DATA_LENGTH (1ul << (PDU_HEADER_LENGTH_SIZE * PDU_BYTE_WIDTH))

/* maximum PDU size in bytes */
#define PDU_MAX_LENGTH  (PDU_HEADER_SIZE + PDU_MAX_DATA_LENGTH)

/* maximum service id in a PDU */
#define PDU_MAX_NUM_SERVICES (1ul << (PDU_HEADER_SERVICE_SIZE * PDU_BYTE_WIDTH))

/* maximum opcode in a PDU */
#define PDU_MAX_NUM_OPCODES (1ul << (PDU_HEADER_OPCODE_SIZE * PDU_BYTE_WIDTH))

struct pdu {
  uint8_t service;
  uint8_t opcode;
  uint16_t len;
  unsigned char data[];
} __attribute__((packed));

void
init_pdu(struct pdu* pdu, uint8_t service, uint8_t opcode);

size_t
pdu_size(const struct pdu* pdu);

long
read_pdu_at(const struct pdu* pdu, unsigned long offset, const char* fmt,
            ...);

long
write_pdu_at(struct pdu* pdu, unsigned long offset, const char* fmt, ...);

long
append_to_pdu(struct pdu* pdu, const char* fmt, ...);
