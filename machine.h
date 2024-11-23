#ifndef MACHINE_H
#define MACHINE_H

#include <inttypes.h>
#include "string.h"
#include "hashmap.h"

#define READ_BUFFER_SIZE 0x10000000
#define WRITE_BUFFER_SIZE 0x10000000
#define AUX_SIZE 0x100000
#define ERROR_BUFFER 0x80
#define ARENA_SIZE READ_BUFFER_SIZE+WRITE_BUFFER_SIZE+AUX_SIZE+ERROR_BUFFER
#define TOKEN_MAX 64

typedef uint64_t word;
typedef uint8_t byte;

typedef enum {
	NOP=0,
	LDS, LDB, LDA, LDI,
	STS, STA, STB,
	MOV, SWP,
	ADS, SUS, MUS, DIS, MOS, ANS, ORS, SLS, SRS, XRS,
	ADI, SUI, MUI, DII, MOI, ANI, ORI, SLI, SRI, XRI,
	ADD, SUB, MUL, DIV, MOD, AND, OR, SHL, SHR, XOR,
	INV, COM, INI, COI,
	CMP, CMS,
	RET, REI, RES, REB,
	ESC,
	CAL,
	PSH, PSS, PSB,
	POP,
	BNC, BNE, BEQ, BLT, BGT, BLE, BGE,
	JMP, JNE, JEQ, JLT, JGT, JLE, JGE,
	INT, INR,
	OPCODE_COUNT
} OPCODE;

MAP_DEF(OPCODE)

typedef enum {
	IP, SP, FP, SR, LR, CR,
	R0, R1, R2, R3, R4, R5, R6, R7, R8, R9,
	REGISTER_COUNT
} REGISTER;

MAP_DEF(REGISTER)

#define SEP_CHAR '['

typedef enum {
	L16,
	LM16,
	RM16,
	R16,
	FULL,
	HALF,
	LO,
	HI,
	PARTITION_COUNT
} REG_PARTITION;

MAP_DEF(REG_PARTITION)

typedef enum {
	ZERO=1,
	CARRY=2,
	SIGN=4,
	OVER=8
} STATUS_FLAG;

typedef struct {
	byte* text;
	word size;
	word i;
} string;

typedef struct label_chain label_chain;

typedef struct label_chain {
	union {
		struct {
			label_chain* next;
			word ref_location;
		} pending;
		struct {
			word address;
		} filled;
	} data;
	enum {
		PENDING_LINK,
		FILLED_LINK
	} tag;
} label_chain;

MAP_DEF(label_chain)

typedef struct {
	string str;
	string buf;
	OPCODE_map opmap;
	REGISTER_map regmap;
	REG_PARTITION_map partmap;
	label_chain_map chain;
	pool* mem;
	char* err;
} compiler;

#endif
